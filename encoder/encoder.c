#include "encoder.h"

static int32_t total_count; /* 累计计数 (32位, 补偿16位定时器溢出) */

/**
 * @brief  初始化编码器
 * @note   调用 HAL_TIM_Encoder_Start 启动编码器模式
 *         必须在 CubeMX 已配置好定时器后调用
 *         调用后定时器开始计数, 可通过 Encoder_Get/GetTotal 读取
 */
void Encoder_Init(void)
{
    total_count = 0;
    HAL_TIM_Encoder_Start(&ENC_TIM, TIM_CHANNEL_ALL);
}

/**
 * @brief  读取编码器增量并清零 CNT
 * @return 本次增量值 (int16_t, 正转正值/反转负值)
 * @note   每次调用后定时器 CNT 归零, 适合周期性测速调用
 *         内部自动累加到 32 位 total_count, 同时更新累计值
 *         调用间隔建议 >= 1ms, 避免增量溢出 (16位CNT最大±32767)
 *         例: 每隔 10ms 调用, dt_s=0.01 传入 Encoder_GetSpeedRPM
 */
int16_t Encoder_Get(void)
{
    int16_t delta = (int16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM);
    __HAL_TIM_SET_COUNTER(&ENC_TIM, 0);
    total_count += delta;
    return delta;
}

/**
 * @brief  读取累计计数值 (不清零)
 * @return 32位累计计数, 正转累加/反转累减
 * @note   适合位置控制、统计总圈数
 *         累计范围: -2147483648 ~ 2147483647 (约 ±162万圈@1320计数/圈)
 */
int32_t Encoder_GetTotal(void)
{
    return total_count;
}

/**
 * @brief  清零累计值和定时器 CNT
 * @note   通常在位置环开始前调用, 初始化位置参考点
 */
void Encoder_ClearTotal(void)
{
    total_count = 0;
    __HAL_TIM_SET_COUNTER(&ENC_TIM, 0);
}

/**
 * @brief  读取输出轴转速 (RPM)
 * @param  dt_s 两次调用之间的时间间隔 (单位: 秒)
 * @return 输出轴转速 (RPM, 正值正转/负值反转)
 * @note   内部会调用 Encoder_Get() 读增量并归零
 *         转速 = (增量 / 一圈计数) / dt_s × 60
 *         例: 10ms 控制周期 → dt_s = 0.01f
 *         如果增量=22, 一圈=1320: RPM = (22/1320)/0.01×60 = 100 RPM
 */
float Encoder_GetSpeedRPM(float dt_s)
{
    int16_t delta = Encoder_Get();
    return (delta / ENC_COUNTS_PER_REV) / dt_s * 60.0f;
}

/**
 * @brief  读取累计圈数 (输出轴)
 * @return 输出轴累计转动圈数 (正转正值/反转负值)
 * @note   圈数 = 累计计数 / 一圈计数
 */
float Encoder_GetRevolutions(void)
{
    return (float)total_count / ENC_COUNTS_PER_REV;
}

