#ifndef __KEY_H
#define __KEY_H

/***
 *  @file    key.h
 *  @brief   按键驱动（HAL库 / 可复用 / 消抖 / 松手检测）
 *  @version V1.0
 *
 *  【如何复用】
 *    1. 将此 key.h / key.c 加入你的工程
 *    2. 在 CubeMX 中配置按键引脚为 GPIO_Input + 上拉 (按键→GND)
 *    3. 修改下方【硬件配置区】的宏，匹配你的按键引脚
 *    4. 在 1ms 定时中断中调用 Key_Tick() 实现 20ms 消抖
 *    5. 主循环中调用 Key_GetNum() 读取按键事件
 *
 *  【消抖原理】
 *    Key_Tick() 每 1ms 被调用，内部计次 20 次 (20ms) 读取一次按键状态
 *    检测松手瞬间 → 返回键码 (只在松手时触发一次, 实现防抖 + 不重复)
 ***/

#include "main.h"

/*==============================================================================
 *               【硬件配置区】—— 更换引脚或增减按键时修改这里
 *============================================================================*/
#define KEY1_PORT  GPIOB               /* 按键1 GPIO端口  */
#define KEY1_PIN   GPIO_PIN_10         /* 按键1 GPIO引脚  */

#define KEY2_PORT  GPIOB               /* 按键2 GPIO端口  */
#define KEY2_PIN   GPIO_PIN_11         /* 按键2 GPIO引脚  */

#define KEY3_PORT  GPIOA               /* 按键3 GPIO端口  */
#define KEY3_PIN   GPIO_PIN_11         /* 按键3 GPIO引脚  */

#define KEY4_PORT  GPIOA               /* 按键4 GPIO端口  */
#define KEY4_PIN   GPIO_PIN_12         /* 按键4 GPIO引脚  */

#define KEY_DEBOUNCE_MS  20            /* 消抖时间 (ms)    */
/*============================================================================*/

void Key_Init(void);                    /* 按键初始化 (GPIO 在 CubeMX 配好, 此函数可空)  */
uint8_t Key_GetNum(void);              /* 读取键码 (松手时返回 1~4, 无按键返回 0, 读后清零) */
void Key_Tick(void);                    /* 按键扫描 (必须每 1ms 调用一次, 放定时中断内)   */

#endif
