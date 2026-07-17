#ifndef __MOTOR_H
#define __MOTOR_H

/***
 *  @file    motor.h
 *  @brief   直流电机驱动模块（HAL库 / 可复用 / PWM调速 + 方向控制 + 刹车）
 *  @version V1.0
 *
 *  【硬件原理】
 *    H桥电机驱动（如 TB6612 / L298N / 江协PID底板板载驱动）
 *    用两个 GPIO 控制方向 (IN1/IN2):
 *      IN1=1,IN2=0 → 正转    IN1=0,IN2=1 → 反转
 *      IN1=0,IN2=0 → 滑行停   IN1=1,IN2=1 → 刹车
 *
 *  【如何复用】
 *    1. 将此 motor.h / motor.c 加入你的工程
 *    2. 在 CubeMX 中配置:
 *       - 一个定时器通道为 PWM Generation (20kHz, ARR=99 或 999)
 *       - 两个 GPIO_Output 推挽 → 方向控制脚 (IN1/IN2)
 *    3. 修改下方【硬件配置区】的宏，匹配你的定时器和 GPIO
 *    4. 调用 Motor_Init() → Motor_SetPWM(n)
 *
 *  【PWM 范围说明】
 *    +MOTOR_PWM_MAX → 最大正转 (满占空比)
 *    -MOTOR_PWM_MAX → 最大反转
 *    0              → 滑行停止
 *    移植时确保 MOTOR_PWM_MAX == CubeMX 的 ARR
 ***/

#include "main.h"
#include "tim.h"

/*==============================================================================
 *               【硬件配置区】—— 更换电机驱动/定时器时修改这里
 *============================================================================*/
#define MOTOR_PWM_HTIM      htim2               /* PWM 定时器句柄            */
#define MOTOR_PWM_CHANNEL   TIM_CHANNEL_1        /* PWM 输出通道              */
#define MOTOR_PWM_MAX       99                   /* PWM 满占空比 = CubeMX ARR */

#define MOTOR_IN1_PORT      GPIOB               /* 方向脚 IN1 端口           */
#define MOTOR_IN1_PIN       GPIO_PIN_12          /* 方向脚 IN1 引脚           */
#define MOTOR_IN2_PORT      GPIOB               /* 方向脚 IN2 端口           */
#define MOTOR_IN2_PIN       GPIO_PIN_13          /* 方向脚 IN2 引脚           */
/*============================================================================*/

void Motor_Init(void);                                  /* 启动 PWM, 置方向脚为滑行停       */
void Motor_SetPWM(int16_t pwm);                         /* 设置占空比: +正转/-反转, 0=停   */
void Motor_Brake(void);                                 /* 刹车 (IN1=IN2=1, 短接制动)     */
void Motor_Stop(void);                                  /* 滑行停 (IN1=IN2=0, 占空比清零)  */

#endif
