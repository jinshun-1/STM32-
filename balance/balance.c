#include "balance.h"
#include "angle.h"
#include "encoder.h"
#include "motor.h"

static volatile Balance_State s_state = BALANCE_STOPPED;
static volatile int16_t s_zero_raw = ANGLE_CENTER_DEFAULT;
static volatile int16_t s_angle = 0;
static volatile float s_angle_rate = 0.0f;
static volatile float s_cart_speed = 0.0f;
static volatile float s_output = 0.0f;
static float s_prev_angle = 0.0f;

static float clampf(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float absf(float value)
{
    return value >= 0.0f ? value : -value;
}

void Balance_Init(void)
{
    s_state = BALANCE_STOPPED;
    s_zero_raw = ANGLE_CENTER_DEFAULT;
    s_angle = 0;
    s_angle_rate = 0.0f;
    s_cart_speed = 0.0f;
    s_output = 0.0f;
    s_prev_angle = 0.0f;
}

void Balance_CalibrateUpright(void)
{
    uint32_t sum = 0U;

    /* 标定期间让控制中断跳过 ADC，避免 HAL ADC 句柄并发访问 */
    s_state = BALANCE_CALIBRATING;
    Motor_Stop();
    for (uint16_t i = 0; i < BALANCE_CALIBRATION_SAMPLES; ++i)
    {
        sum += Angle_GetRaw();
        HAL_Delay(2);
    }

    s_zero_raw = (int16_t)(sum / BALANCE_CALIBRATION_SAMPLES);
    s_angle = 0;
    s_angle_rate = 0.0f;
    s_prev_angle = 0.0f;
    s_state = BALANCE_STOPPED;
}

uint8_t Balance_Start(void)
{
    /* 单次转换仅数微秒，短临界区可避免 TIM1 中断抢占 ADC 操作 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int16_t angle = (int16_t)((int32_t)Angle_GetRaw() - s_zero_raw);

    if (angle > (int16_t)BALANCE_ANGLE_ARM_LIMIT ||
        angle < -(int16_t)BALANCE_ANGLE_ARM_LIMIT)
    {
        s_state = BALANCE_FAULT_ANGLE;
        Motor_Stop();
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    Encoder_ClearTotal();
    s_angle = angle;
    s_prev_angle = (float)angle;
    s_angle_rate = 0.0f;
    s_cart_speed = 0.0f;
    s_output = 0.0f;
    s_state = BALANCE_RUNNING;
    if (primask == 0U) { __enable_irq(); }
    return 1U;
}

void Balance_Stop(void)
{
    s_state = BALANCE_STOPPED;
    s_output = 0.0f;
    Motor_Stop();
}

void Balance_ControlStep(int16_t encoder_delta)
{
    if (s_state == BALANCE_CALIBRATING)
    {
        s_output = 0.0f;
        Motor_Stop();
        return;
    }

    int16_t raw_angle = (int16_t)((int32_t)Angle_GetRaw() - s_zero_raw);
    float angle = BALANCE_ANGLE_SIGN * (float)raw_angle;
    float angle_rate_raw = (angle - s_prev_angle) / BALANCE_DT;
    float cart_speed_raw = (float)encoder_delta / BALANCE_DT;

    /* 电位器和编码器微分都做低通，降低 ADC 噪声放大 */
    s_angle_rate += 0.22f * (angle_rate_raw - s_angle_rate);
    s_cart_speed += 0.25f * (cart_speed_raw - s_cart_speed);
    s_prev_angle = angle;
    s_angle = (int16_t)angle;

    if (s_state != BALANCE_RUNNING)
    {
        s_output = 0.0f;
        Motor_Stop();
        return;
    }

    float position = (float)Encoder_GetTotal();
    if (absf(angle) > BALANCE_ANGLE_TRIP_LIMIT)
    {
        s_state = BALANCE_FAULT_ANGLE;
        s_output = 0.0f;
        Motor_Stop();
        return;
    }
    if (absf(position) > BALANCE_POSITION_LIMIT)
    {
        s_state = BALANCE_FAULT_POSITION;
        s_output = 0.0f;
        Motor_Stop();
        return;
    }

    /*
     * ==================== 倒立摆 LQR 状态反馈 ====================
     *
     *  PWM = Kθ×θ + Kω×θdot - Kx×x - Kv×xdot
     *
     *  ┌─────────────────┬────────────────────────────────────┐
     *  │  变量            │  调参口诀                          │
     *  ├─────────────────┼────────────────────────────────────┤
     *  │  Kθ  (角度)      │  主力扶正。松手秒倒→加、猛甩→减    │
     *  │  Kω  (角速度)    │  阻尼消抖。高频抖→加、迟钝→减      │
     *  │ -Kx  (位置)      │  定住小车。漂移跑远→加             │
     *  │ -Kv  (速度)      │  减速稳车。来回晃→加               │
     *  └─────────────────┴────────────────────────────────────┘
     *
     *  【调参顺序】 Kθ → Kω → Kv → Kx   每次只改一个, 每次不超过20%
     *  【方向错误】 摆杆右偏但小车左跑 → 取反 BALANCE_MOTOR_SIGN
     */
    float output = BALANCE_K_ANGLE       * angle           /* ① Kθ: 角度越大出力越大       */
                 + BALANCE_K_ANGLE_RATE  * s_angle_rate    /* ② Kω: 角速度越大刹车越猛     */
                 - BALANCE_K_POSITION    * position        /* ③ Kx: 位置越远拉力越大(拉回) */
                 - BALANCE_K_VELOCITY    * s_cart_speed;   /* ④ Kv: 速度越大阻尼越大(减速) */

    output = clampf(output, -BALANCE_PWM_MAX, BALANCE_PWM_MAX);
    s_output = BALANCE_MOTOR_SIGN * output;
    Motor_SetPWM((int16_t)s_output);
}

Balance_State Balance_GetState(void) { return s_state; }
int16_t Balance_GetAngle(void) { return s_angle; }
float Balance_GetAngleRate(void) { return s_angle_rate; }
float Balance_GetCartSpeed(void) { return s_cart_speed; }
float Balance_GetOutput(void) { return s_output; }
int16_t Balance_GetZeroRaw(void) { return s_zero_raw; }
