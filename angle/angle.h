#ifndef __ANGLE_H
#define __ANGLE_H

/***
 *  @file    angle.h
 *  @brief   角度传感器驱动（电位器/ADC / HAL库 / 可复用）
 *  @version V1.0
 *
 *  【传感器类型】
 *    旋转电位器 (ADC 读取，0~4095)
 *
 *  【校准参数】
 *    摆杆自然下垂时读取 CENTER_RAW，即为中立点
 *    左右摆动的有效范围由 CENTER_RANGE 决定
 *
 *  【如何复用】
 *    1. 在 CubeMX 中配置 ADC 通道 (单次转换, 软件触发, 右对齐)
 *    2. 修改下方 ADC_HANDLE 和 ADC_CHANNEL 宏
 *    3. 摆杆垂直下垂时记下 ADC 读数, 填入 CENTER_RAW
 *    4. 调用 Angle_Init() → Angle_GetValue() / Angle_Get()
 ***/

#include "main.h"
#include "adc.h"

/*==============================================================================
 *               【硬件配置区】—— 更换 ADC 通道或校准值时修改
 *============================================================================*/
#define ANGLE_ADC              hadc1               /* ADC 句柄 (CubeMX 生成)       */
#define ANGLE_CHANNEL          ADC_CHANNEL_8       /* ADC 通道 (PB0 = ADC1_IN8)    */
#define ANGLE_CENTER_DEFAULT   2010                /* 默认直立零点，K1可重新校准   */
#define ANGLE_RANGE            500                 /* 通用超限范围                 */
/*============================================================================*/

void Angle_Init(void);                          /* 初始化 ADC (CubeMX 已配)      */
uint16_t Angle_GetRaw(void);                    /* 读取原始 ADC 值 (0~4095)       */
int16_t Angle_FromRaw(uint16_t raw, int16_t center); /* 原始ADC转相对角度           */
int16_t Angle_Get(void);                        /* 相对默认零点的角度              */
uint8_t Angle_IsOutOfRange(void);               /* 是否超出允许范围                */

#endif
