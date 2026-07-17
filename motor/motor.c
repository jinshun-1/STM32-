#include "motor.h"

/**
 * @brief  电机初始化
 * @note   启动 PWM 输出, 方向引脚置低 → 滑行停状态
 *         CubeMX 已配好 TIM2_CH1 PWM 和 PB12/PB13 输出
 */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&MOTOR_PWM_HTIM, MOTOR_PWM_CHANNEL);
    Motor_Stop();
}

/**
 * @brief  设置电机 PWM (带方向)
 * @param  pwm 带符号占空比: +正转 / -反转 / 0=停
 *             范围: -MOTOR_PWM_MAX ~ +MOTOR_PWM_MAX
 * @note   例: Motor_SetPWM(50)  → 正转 50% 占空比
 *              Motor_SetPWM(-80) → 反转 80% 占空比
 *              Motor_SetPWM(0)   → 滑行停
 *
 *         方向控制:
 *           正转: IN1=1, IN2=0
 *           反转: IN1=0, IN2=1
 *           停止: IN1=0, IN2=0
 *
 *         通过 __HAL_TIM_SET_COMPARE 设置 CCR 值
 */
void Motor_SetPWM(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)  pwm = MOTOR_PWM_MAX;
    if (pwm < -MOTOR_PWM_MAX) pwm = -MOTOR_PWM_MAX;

    if (pwm > 0)
    {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_PWM_CHANNEL, pwm);
    }
    else if (pwm < 0)
    {
        HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_PWM_CHANNEL, -pwm);
    }
    else
    {
        Motor_Stop();
    }
}

/**
 * @brief  刹车 (IN1=IN2=1, 电机绕组短路制动, 快速停止)
 * @note   比滑行停更快, 但有瞬间大电流
 */
void Motor_Brake(void)
{
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_PWM_CHANNEL, 0);
}

/**
 * @brief  滑行停 (IN1=IN2=0, 占空比清零, 电机惯性转动)
 * @note   安全停止方式, 无制动电流
 */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_PWM_CHANNEL, 0);
}
