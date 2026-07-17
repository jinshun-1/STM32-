#ifndef __ENCODER_H
#define __ENCODER_H

/***
 *  @file    encoder.h
 *  @brief   增量式编码器驱动（HAL库 / 可复用）
 *  @version V1.0
 *
 *  【支持的编码器型号】
 *    JGB37-520 直流减速电机 霍尔编码器（默认配置）
 *    参数: 11 PPR, 30:1 减速比, 4倍频 → 输出轴一圈 1320 计数
 *
 *  【如何复用】
 *    1. 将此 encoder.h / encoder.c 加入你的工程
 *    2. 在 CubeMX 中配置一个定时器为 Encoder Mode (TI1 and TI2)
 *    3. 在 CubeMX 中配好编码器对应的两个 GPIO 引脚
 *    4. 修改下方【硬件配置区】的宏，匹配你的定时器和编码器参数
 *    5. 调用 Encoder_Init() 初始化，然后调用其他 API 读取数据
 *
 *  【针对其他编码器型号的移植示例】
 *    如果你的编码器是 13线 / 100:1 减速比，只需修改:
 *      #define ENC_PPR        13.0f
 *      #define ENC_GEAR_RATIO 100.0f
 ***/

#include "main.h"
#include "tim.h"

/*==============================================================================
 *               【硬件配置区】—— 更换编码器型号或定时器时修改这里
 *============================================================================*/
#define ENC_TIM             htim3           /* 编码器使用的定时器句柄 (CubeMX生成的htimX) */
#define ENC_PPR             11.0f           /* 编码器物理线数 (每圈脉冲数, 霍尔编码器常见11/13线) */
#define ENC_GEAR_RATIO      30.0f           /* 减速电机减速比 (电机轴转速 / 输出轴转速) */
#define ENC_MULTIPLE        4.0f            /* 正交倍频 (TI1&TI2 → 4倍, 不改) */
#define ENC_COUNTS_PER_REV  (ENC_PPR * ENC_MULTIPLE * ENC_GEAR_RATIO)  /* 输出轴一圈的计数值 = 1320 */
/*============================================================================*/

void Encoder_Init(void);                      /* 启动编码器 (HAL_TIM_Encoder_Start)        */
int16_t Encoder_Get(void);                    /* 读增量并清零 (适合周期性测速调用)          */
int32_t Encoder_GetTotal(void);               /* 读累计值,不清零 (适合位置/圈数统计)        */
void Encoder_ClearTotal(void);                /* 清零累计值和定时器CNT                     */
float Encoder_GetSpeedRPM(float dt_s);        /* 读输出轴转速 (RPM), dt_s=两次调用间隔(秒)  */
float Encoder_GetRevolutions(void);           /* 读累计圈数 (输出轴)                       */

#endif
