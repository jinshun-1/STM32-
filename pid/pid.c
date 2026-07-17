#include "pid.h"

/* ==================== 工具函数 ==================== */

static float clamp(float val, float min, float max)
{
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

/* ==================== 初始化 ==================== */

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
    pid->prev_output = 0;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = (out_max - out_min) * 0.5f;  // 积分限幅默认输出范围的一半
}

void PID_Tune(PID_Controller *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_measurement = 0;
    pid->prev_output = 0;
}

/* ==================== 位置式PID ==================== */

float PID_Compute(PID_Controller *pid, float target, float actual, float dt_s)
{
    float error = target - actual;                    // 当前误差

    // ── P 比例项 ──
    float p_out = pid->kp * error;

    // ── I 积分项 (抗积分饱和: 输出已饱和时不继续积分) ──
    pid->integral += error * dt_s;
    pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    float i_out = pid->ki * pid->integral;

    // ── D 微分项 (对误差微分) ──
    float derivative = (error - pid->prev_error) / dt_s;
    float d_out = pid->kd * derivative;

    pid->prev_error = error;

    // ── 输出限幅 ──
    return clamp(p_out + i_out + d_out, pid->out_min, pid->out_max);
}

/* ==================== 增量式PID ==================== */

/**
 * 位置式连续两次输出的差值:
 *   Δu(k) = u(k) - u(k-1)
 *         = Kp*(e(k)-e(k-1)) + Ki*e(k)*dt + Kd*(e(k)-2*e(k-1)+e(k-2))/dt
 *
 * 由于只输出增量, 不需要存储积分历史, 天然抗饱和
 */
float PID_ComputeInc(PID_Controller *pid, float target, float actual)
{
    float error = target - actual;

    // 增量式公式 (dt隐含在积分项中, 需调参时调整Ki)
    float p_inc = pid->kp * (error - pid->prev_error);
    float i_inc = pid->ki * error * 0.01f;   // 配合10ms调用周期
    float d_inc = pid->kd * (error - 2 * pid->prev_error + pid->prev_measurement);

    pid->prev_measurement = pid->prev_error;  // e(k-2) ← e(k-1)
    pid->prev_error = error;                  // e(k-1) ← e(k)

    float inc = p_inc + i_inc + d_inc;

    // 增量限幅
    inc = clamp(inc, pid->out_min, pid->out_max);

    pid->prev_output += inc;
    pid->prev_output = clamp(pid->prev_output, pid->out_min, pid->out_max);

    return inc;
}

/* ==================== 微分先行PID ==================== */

/**
 * 微分项只对测量值求导, 不对目标值:
 *   标准: D = Kd * d(target-actual)/dt      ← 目标突变→微分冲击
 *   先行: D = Kd * d(-actual)/dt = -Kd * d(actual)/dt   ← 平稳
 */
float PID_ComputeDF(PID_Controller *pid, float target, float actual, float dt_s)
{
    float error = target - actual;

    float p_out = pid->kp * error;

    pid->integral += error * dt_s;
    pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    float i_out = pid->ki * pid->integral;

    // 微分: 只对测量值变化率, 不去碰target
    float d_out = -pid->kd * (actual - pid->prev_measurement) / dt_s;

    pid->prev_measurement = actual;
    pid->prev_error = error;

    return clamp(p_out + i_out + d_out, pid->out_min, pid->out_max);
}

/* ==================== 前馈PID ==================== */

/**
 * 前馈 = 预测补偿, 在误差出现之前就加力
 * 例如: 已知云台要以 30°/s 旋转, feedforward = 30 * Kv (速度前馈系数)
 */
float PID_ComputeFF(PID_Controller *pid, float target, float actual, float dt_s, float feedforward)
{
    float pid_out = PID_Compute(pid, target, actual, dt_s);
    return clamp(pid_out + feedforward, pid->out_min, pid->out_max);
}

/* ==================== 串级PID ==================== */

/**
 * 外环(如角度) → 计算误差 → PID输出作为内环目标(如速度)
 * 内环(如速度) → 计算误差 → PID输出最终PWM
 *
 * 比单环PID更稳, 因为内环先抑制了速度扰动
 */
float PID_ComputeCascade(PID_Controller *outer, PID_Controller *inner,
                         float outer_target, float outer_actual,
                         float inner_actual, float dt_s)
{
    // 外环: 角度→目标速度
    float inner_target = PID_Compute(outer, outer_target, outer_actual, dt_s);

    // 内环: 速度→PWM
    return PID_Compute(inner, inner_target, inner_actual, dt_s);
}
