#include "key.h"

static uint8_t Key_Num;  /* 按键键码缓存 (松手时填入, GetNum 读后清零) */

/**
 * @brief  按键初始化
 * @note   GPIO 已在 CubeMX 中配置为 Input + 上拉, 此处无需额外操作
 */
void Key_Init(void)
{
}

/**
 * @brief  读取按键键码 (非阻塞, 读后清零)
 * @return 1-4 对应 K1-K4 松手事件, 0 表示无按键
 * @note   每次按键松手只触发一次 (由 Key_Tick 消抖保证)
 *         返回值读一次后清零, 再次调用返回 0
 *
 *         使用示例:
 *           uint8_t key = Key_GetNum();
 *           if (key == 1) { ... }  // K1 被按下又松开
 */
uint8_t Key_GetNum(void)
{
    uint8_t temp;
    if (Key_Num)
    {
        temp = Key_Num;
        Key_Num = 0;
        return temp;
    }
    return 0;
}

/**
 * @brief  读取当前按键状态 (内部函数, 不消抖)
 * @return 1-4 对应按下的按键, 0 表示无按键按下
 * @note   直接读 GPIO 电平, 低电平 = 按下 (上拉输入, 按键→GND)
 */
static uint8_t Key_GetState(void)
{
    if (HAL_GPIO_ReadPin(KEY1_PORT, KEY1_PIN) == GPIO_PIN_RESET) return 1;
    if (HAL_GPIO_ReadPin(KEY2_PORT, KEY2_PIN) == GPIO_PIN_RESET) return 2;
    if (HAL_GPIO_ReadPin(KEY3_PORT, KEY3_PIN) == GPIO_PIN_RESET) return 3;
    if (HAL_GPIO_ReadPin(KEY4_PORT, KEY4_PIN) == GPIO_PIN_RESET) return 4;
    return 0;
}

/**
 * @brief  按键扫描 + 消抖 (必须每 1ms 调用一次)
 * @note   放在 1ms 定时器中断 (如 TIM1) 中调用
 *         原理: 每 20ms 读一次状态, 检测松手瞬间 (上次按下且本次松开)
 *
 *         使用示例 (在 TIM1_UP_IRQHandler 中):
 *           void TIM1_UP_IRQHandler(void) {
 *               Key_Tick();  // 每1ms调用, 内部20ms消抖
 *           }
 */
void Key_Tick(void)
{
    static uint8_t count;
    static uint8_t curr, prev;

    count++;
    if (count >= KEY_DEBOUNCE_MS)
    {
        count = 0;
        prev = curr;
        curr = Key_GetState();
        if (curr == 0 && prev != 0)   /* 松手瞬间 */
        {
            Key_Num = prev;
        }
    }
}
