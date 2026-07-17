#ifndef __MPU6050_H
#define __MPU6050_H

/**
 * ==================== MPU6050 驱动 (STM32F4 HAL) ====================
 *
 * 硬件连接: VCC→3.3V, GND→GND, SCL→I2C_SCL, SDA→I2C_SDA, AD0→GND
 *
 * CubeMX配置:
 *   开启一个I2C, 模式选 I2C, Speed=Fast Mode, Freq=400KHz
 *
 * 使用步骤:
 *   1. MPU6050_Init(&hi2c, ACCEL_2G, GYRO_2000DPS, DLPF_5HZ);
 *   2. MPU6050_ReadScaled(&hi2c, &data);   // 读物理量
 *   3. 互补滤波获取姿态角:
 *        MPU6050_Filter_Init(&cf, 0.98f);
 *        while(1) {
 *            float dt = (HAL_GetTick() - tick) * 0.001f;
 *            MPU6050_ReadScaled(&hi2c, &data);
 *            MPU6050_Filter_Update(&cf, &data, dt);
 *        }
 *   4. (可选) 滑动平均平滑单轴数据:
 *        val = MPU6050_AvgFilter_Update(&avgf, raw_val);
 * ===================================================================
 */

#include "stm32f4xx_hal.h"
#include <math.h>

/* ==================== I2C 地址 ==================== */
#define MPU6050_ADDR        0xD0    // AD0=GND 时的7位地址左移1位
#define MPU6050_ADDR_WRITE  (MPU6050_ADDR)
#define MPU6050_ADDR_READ   (MPU6050_ADDR | 0x01)

/* ==================== 寄存器地址 ==================== */
#define MPU6050_REG_SMPLRT_DIV     0x19    // 采样率分频: Fsample = Fgyro / (1 + DIV)
#define MPU6050_REG_CONFIG         0x1A    // 数字低通滤波器带宽配置
#define MPU6050_REG_GYRO_CONFIG    0x1B    // 陀螺仪自检+量程配置
#define MPU6050_REG_ACCEL_CONFIG   0x1C    // 加速度计自检+量程+高通滤波
#define MPU6050_REG_ACCEL_XOUT_H   0x3B    // 加速度X高字节 (共14字节连续: AccelXYZ + Temp + GyroXYZ)
#define MPU6050_REG_ACCEL_XOUT_L   0x3C
#define MPU6050_REG_ACCEL_YOUT_H   0x3D
#define MPU6050_REG_ACCEL_YOUT_L   0x3E
#define MPU6050_REG_ACCEL_ZOUT_H   0x3F
#define MPU6050_REG_ACCEL_ZOUT_L   0x40
#define MPU6050_REG_TEMP_OUT_H     0x41    // 温度高字节
#define MPU6050_REG_TEMP_OUT_L     0x42
#define MPU6050_REG_GYRO_XOUT_H    0x43    // 陀螺仪X高字节
#define MPU6050_REG_GYRO_XOUT_L    0x44
#define MPU6050_REG_GYRO_YOUT_H    0x45
#define MPU6050_REG_GYRO_YOUT_L    0x46
#define MPU6050_REG_GYRO_ZOUT_H    0x47
#define MPU6050_REG_GYRO_ZOUT_L    0x48
#define MPU6050_REG_PWR_MGMT_1     0x6B    // 电源管理: 写0x00唤醒
#define MPU6050_REG_WHO_AM_I       0x75    // 设备ID: 应返回0x68

/* ==================== 陀螺仪量程枚举 ==================== */
typedef enum {
    MPU6050_GYRO_250DPS  = 0x00,    // ±250°/s,  灵敏度 131.0 LSB/°/s
    MPU6050_GYRO_500DPS  = 0x08,    // ±500°/s,  灵敏度  65.5 LSB/°/s
    MPU6050_GYRO_1000DPS = 0x10,    // ±1000°/s, 灵敏度  32.8 LSB/°/s
    MPU6050_GYRO_2000DPS = 0x18     // ±2000°/s, 灵敏度  16.4 LSB/°/s
} MPU6050_GyroRange;

/* ==================== 加速度计量程枚举 ==================== */
typedef enum {
    MPU6050_ACCEL_2G  = 0x00,       // ±2g,  灵敏度 16384 LSB/g
    MPU6050_ACCEL_4G  = 0x08,       // ±4g,  灵敏度  8192 LSB/g
    MPU6050_ACCEL_8G  = 0x10,       // ±8g,  灵敏度  4096 LSB/g
    MPU6050_ACCEL_16G = 0x18        // ±16g, 灵敏度  2048 LSB/g
} MPU6050_AccelRange;

/* ==================== 数字低通滤波器(DLPF)枚举 ==================== */
typedef enum {
    MPU6050_DLPF_256HZ = 0x00,      // 陀螺仪带宽256Hz, 延时0.98ms
    MPU6050_DLPF_188HZ = 0x01,      // 陀螺仪带宽188Hz, 延时1.9ms
    MPU6050_DLPF_98HZ  = 0x02,      // 陀螺仪带宽 98Hz, 延时2.8ms
    MPU6050_DLPF_42HZ  = 0x03,      // 陀螺仪带宽 42Hz, 延时4.8ms
    MPU6050_DLPF_20HZ  = 0x04,      // 陀螺仪带宽 20Hz, 延时8.3ms
    MPU6050_DLPF_10HZ  = 0x05,      // 陀螺仪带宽 10Hz, 延时13.4ms
    MPU6050_DLPF_5HZ   = 0x06       // 陀螺仪带宽  5Hz, 延时18.6ms
} MPU6050_DLPF;

/* ==================== 原始数据(int16_t ADC值) ==================== */
typedef struct {
    int16_t ax, ay, az;     // 加速度原始值
    int16_t gx, gy, gz;     // 陀螺仪原始值
    int16_t temp;           // 温度原始值
} MPU6050_RawData;

/* ==================== 转换后物理量(float) ==================== */
typedef struct {
    float ax, ay, az;       // 加速度 (g)
    float gx, gy, gz;       // 角速度 (°/s)
    float temp;             // 温度 (°C)
} MPU6050_ScaledData;

/* ==================== 姿态角 ==================== */
typedef struct {
    float roll;             // 俯仰角 (°)  X轴
    float pitch;            // 横滚角 (°)  Y轴
    float yaw;              // 偏航角 (°)  Z轴 (无磁力计会漂移)
} MPU6050_Angle;

/* ==================== 滑动平均滤波器(20点窗口) ==================== */
typedef struct {
    float buf[20];
    uint8_t idx;
    uint8_t count;
    float sum;
} MPU6050_AvgFilter;

/* ==================== 互补滤波器状态 ==================== */
typedef struct {
    MPU6050_Angle angle;    // 当前姿态角
    uint32_t last_time;     // 内部标记(勿手动修改)
    float alpha;            // 融合系数: 0.9~0.99, 越大越信陀螺仪
} MPU6050_Filter;

/* ==================== 基础驱动函数 ==================== */

/**
 * @brief  初始化MPU6050 (唤醒 + 配置量程/滤波器)
 * @param  hi2c         HAL I2C句柄指针, 如 &hi2c2
 * @param  accel_range  加速度计量程, 如 MPU6050_ACCEL_2G
 * @param  gyro_range   陀螺仪量程,   如 MPU6050_GYRO_2000DPS
 * @param  dlpf         低通滤波器带宽, 如 MPU6050_DLPF_5HZ
 */
void MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_AccelRange accel_range, MPU6050_GyroRange gyro_range, MPU6050_DLPF dlpf);

/**
 * @brief  读取原始ADC值(6轴+温度, 14字节))
 * @param  hi2c  HAL I2C句柄指针
 * @param  raw   输出: 原始数据
 */
void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData *raw);

/**
 * @brief  读取转换后的物理量(加速度:g, 角速度:°/s, 温度:°C)
 * @param  hi2c  HAL I2C句柄指针
 * @param  data  输出: 物理量
 */
void MPU6050_ReadScaled(I2C_HandleTypeDef *hi2c, MPU6050_ScaledData *data);

/**
 * @brief  读取WHO_AM_I寄存器验证通信
 * @param  hi2c  HAL I2C句柄指针
 * @return 设备ID, 正常返回 0x68
 */
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c);

/* ==================== 校准函数 ==================== */

/**
 * @brief  陀螺仪零点校准
 * @param  hi2c    HAL I2C句柄指针
 * @param  samples 采样次数, 推荐150~300, 越多越准但越慢
 *
 * 使用方法:
 *   放平传感器 → 调用 MPU6050_Calibrate(&hi2c, 200) → 等待完成
 *   之后的 MPU6050_ReadScaled() 会自动减去零点偏移
 *
 * 原理:
 *   静止时陀螺仪输出应接近0, 但实际有几十到几百LSB的偏差
 *   校准后采集N个样本取平均值作为零点, 后续每帧减去此偏差
 */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples);

/* ==================== 滤波算法函数 ==================== */

/**
 * @brief  初始化互补滤波器
 * @param  f     滤波器结构体指针
 * @param  alpha 融合系数(0.9~0.99)
 *               alpha越大越信任陀螺仪 → 响应快但会漂移
 *               alpha越小越信任加速度计 → 稳定但噪音大
 *               推荐: 0.98 (平衡小车常用)
 *
 * 互补滤波公式:
 *   angle = alpha * (angle + gyro * dt) + (1-alpha) * acc_angle
 *   - 陀螺仪积分: 短期精确, 长期漂移
 *   - 加速度计:   长期准确, 短期噪声大(受振动影响)
 */
void MPU6050_Filter_Init(MPU6050_Filter *f, float alpha);

/**
 * @brief  更新互补滤波器, 计算当前姿态角
 * @param  f     滤波器状态
 * @param  data  当前帧物理量 (由 MPU6050_ReadScaled 获取)
 * @param  dt_s  距离上一帧的时间间隔(秒)
 *               计算方式: (HAL_GetTick() - last_tick) * 0.001f
 *
 * 使用示例:
 *   MPU6050_Filter cf;
 *   MPU6050_Filter_Init(&cf, 0.98f);
 *   uint32_t last = HAL_GetTick();
 *   while(1) {
 *       MPU6050_ScaledData d;
 *       MPU6050_ReadScaled(&hi2c, &d);
 *       float dt = (HAL_GetTick() - last) * 0.001f;
 *       last = HAL_GetTick();
 *       MPU6050_Filter_Update(&cf, &d, dt);
 *       printf("Roll:%.1f Pitch:%.1f\n", cf.angle.roll, cf.angle.pitch);
 *   }
 */
void MPU6050_Filter_Update(MPU6050_Filter *f, MPU6050_ScaledData *data, float dt_s);

/**
 * @brief  初始化滑动平均滤波器
 * @param  f  滤波器结构体指针
 *
 * 用于平滑单个轴的数据(如加速度、陀螺仪某轴),
 * 窗口大小20, 适合去除高频毛刺
 */
void MPU6050_AvgFilter_Init(MPU6050_AvgFilter *f);

/**
 * @brief  滑动平均滤波, 输入新值返回平滑后的值
 * @param  f   滤波器状态
 * @param  val 新采样值
 * @return 20点平均后的值
 *
 * 使用示例:
 *   MPU6050_AvgFilter ax_f;
 *   MPU6050_AvgFilter_Init(&ax_f);
 *   float smooth_ax = MPU6050_AvgFilter_Update(&ax_f, data.ax);
 */
float MPU6050_AvgFilter_Update(MPU6050_AvgFilter *f, float val);

#endif
