#include "angle.h"

/**
 * @brief  初始化角度传感器
 * @note   ADC 已在 CubeMX 中配置并调用 MX_ADC1_Init(), 此处无需额外操作
 */
void Angle_Init(void)
{
    /* STM32F1 上电后执行一次 ADC 校准 */
    HAL_ADCEx_Calibration_Start(&ANGLE_ADC);
}

/**
 * @brief  读取原始 ADC 值
 * @return 0~4095, 对应 0V~3.3V
 * @note   软件触发单次转换, 阻塞等待完成
 *         对应摆杆角度: 值越大→摆杆越偏一侧
 */
uint16_t Angle_GetRaw(void)
{
    HAL_ADC_Start(&ANGLE_ADC);
    if (HAL_ADC_PollForConversion(&ANGLE_ADC, 10) == HAL_OK)
    {
        return (uint16_t)HAL_ADC_GetValue(&ANGLE_ADC);
    }
    return ANGLE_CENTER_DEFAULT;
}

/**
 * @brief  读取角度偏移值
 * @return 相对中立点的偏移量 (正=右偏/负=左偏, 取决于电位器方向)
 * @note   0 表示摆杆垂直下垂 (中立点)
 *         正值表示偏离中心一侧, 负值表示偏离另一侧
 *         用于 PID 控制时: 目标=0, 反馈=Angle_Get()
 *
 *         示例: 中心=2010, 读数=2200 → 偏移=+190 (偏右)
 *               中心=2010, 读数=1800 → 偏移=-210 (偏左)
 */
int16_t Angle_FromRaw(uint16_t raw, int16_t center)
{
    return (int16_t)((int32_t)raw - center);
}

int16_t Angle_Get(void)
{
    return Angle_FromRaw(Angle_GetRaw(), ANGLE_CENTER_DEFAULT);
}

/**
 * @brief  角度是否超出允许范围 (安全保护)
 * @return 1=超限 (摆杆倒下), 0=正常范围
 * @note   当 |偏移| > ANGLE_RANGE 时判定为超限
 *         倒立摆控制中, 超限应立即关闭电机防止冲撞
 */
uint8_t Angle_IsOutOfRange(void)
{
    int16_t angle = Angle_Get();
    return (angle > ANGLE_RANGE || angle < -ANGLE_RANGE) ? 1 : 0;
}
