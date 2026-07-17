#include "mpu6050.h"

/* ==================== 内部变量 ==================== */

static float accel_scale = 16384.0f;   // 加速度灵敏度 (LSB/g), 初始化时根据量程设置
static float gyro_scale  = 131.0f;     // 陀螺仪灵敏度 (LSB/°/s)
static float gyro_bias[3] = {0};       // 陀螺零点偏移 (°/s), 校准后填充

/* ==================== I2C 底层读写 ==================== */

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

/* ==================== 初始化和数据读取 ==================== */

void MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_AccelRange accel_range, MPU6050_GyroRange gyro_range, MPU6050_DLPF dlpf)
{
    // 1. 唤醒芯片 (退出睡眠模式)
    MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x00);
    HAL_Delay(100);                     // 等待芯片稳定

    // 2. 配置寄存器
    MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07);    // 采样率: 1kHz/(1+7)=125Hz
    MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, dlpf);         // 数字低通滤波
    MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, gyro_range);   // 陀螺仪量程
    MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, accel_range); // 加速度计量程

    // 3. 根据量程设置对应的灵敏度系数
    switch (accel_range) {
    case MPU6050_ACCEL_2G:  accel_scale = 16384.0f; break;
    case MPU6050_ACCEL_4G:  accel_scale =  8192.0f; break;
    case MPU6050_ACCEL_8G:  accel_scale =  4096.0f; break;
    case MPU6050_ACCEL_16G: accel_scale =  2048.0f; break;
    }

    switch (gyro_range) {
    case MPU6050_GYRO_250DPS:  gyro_scale = 131.0f; break;
    case MPU6050_GYRO_500DPS:  gyro_scale = 65.5f;  break;
    case MPU6050_GYRO_1000DPS: gyro_scale = 32.8f;  break;
    case MPU6050_GYRO_2000DPS: gyro_scale = 16.4f;  break;
    }
}

void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_RawData *raw)
{
    uint8_t buf[14];
    // 从ACCEL_XOUT_H开始连续读14字节, 依次为: AccelXYZ(6) + Temp(2) + GyroXYZ(6)
    MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    // 高字节在前, 拼接为16位有符号数
    raw->ax   = (int16_t)((buf[0]  << 8) | buf[1]);   // 加速度X
    raw->ay   = (int16_t)((buf[2]  << 8) | buf[3]);   // 加速度Y
    raw->az   = (int16_t)((buf[4]  << 8) | buf[5]);   // 加速度Z
    raw->temp = (int16_t)((buf[6]  << 8) | buf[7]);   // 温度
    raw->gx   = (int16_t)((buf[8]  << 8) | buf[9]);   // 陀螺仪X
    raw->gy   = (int16_t)((buf[10] << 8) | buf[11]);  // 陀螺仪Y
    raw->gz   = (int16_t)((buf[12] << 8) | buf[13]);  // 陀螺仪Z
}

void MPU6050_ReadScaled(I2C_HandleTypeDef *hi2c, MPU6050_ScaledData *data)
{
    MPU6050_RawData raw;
    MPU6050_ReadRaw(hi2c, &raw);

    // 原始值除以灵敏度 → 物理量
    data->ax   = raw.ax / accel_scale;                 // 加速度 (g)
    data->ay   = raw.ay / accel_scale;
    data->az   = raw.az / accel_scale;
    data->gx   = raw.gx / gyro_scale - gyro_bias[0];   // 角速度 (°/s), 扣除零点偏移
    data->gy   = raw.gy / gyro_scale - gyro_bias[1];
    data->gz   = raw.gz / gyro_scale - gyro_bias[2];
    data->temp = raw.temp / 340.0f + 36.53f;            // 温度 (°C)
}

uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t id;
    MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &id, 1);
    return id;  // 正常返回 0x68
}

/* ==================== 陀螺仪零点校准 ==================== */

/**
 * 采集 samples 个样本取平均, 得到陀螺仪静止时的零偏 (°/s)
 * 校准结果存入 gyro_bias[], 之后 ReadScaled() 会自动扣除
 * 要求: 调用时传感器必须保持静止
 */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples)
{
    int32_t sum[3] = {0};                    // 用32位累加防止溢出

    for (uint16_t i = 0; i < samples; i++) {
        MPU6050_RawData raw;
        MPU6050_ReadRaw(hi2c, &raw);
        sum[0] += raw.gx;                    // 累加陀螺仪X轴原始值
        sum[1] += raw.gy;                    // 累加Y轴
        sum[2] += raw.gz;                    // 累加Z轴
        HAL_Delay(2);                        // 间隔2ms, 125Hz采样率足够
    }

    // 平均值除以灵敏度 → °/s 单位的零点偏移
    gyro_bias[0] = (sum[0] / (float)samples) / gyro_scale;
    gyro_bias[1] = (sum[1] / (float)samples) / gyro_scale;
    gyro_bias[2] = (sum[2] / (float)samples) / gyro_scale;
}

/* ==================== 互补滤波 ==================== */

void MPU6050_Filter_Init(MPU6050_Filter *f, float alpha)
{
    f->angle.roll = 0;
    f->angle.pitch = 0;
    f->angle.yaw = 0;
    f->last_time = 0;           // 标记首次调用, Filter_Update 会用加速度计初始化角度
    f->alpha = alpha;           // 融合系数
}

/**
 * 互补滤波核心公式:
 *   angle = alpha * (angle + gyro*dt) + (1-alpha) * acc_angle
 *
 * - 陀螺仪部分: 积分角速度得到角度增量, 短期精确但会漂移
 * - 加速度计部分: 通过重力分量反算角度, 长期准确但受振动影响
 * - alpha=0.98: 98%信任陀螺仪, 2%信任加速度计 (平衡小车常用值)
 */
void MPU6050_Filter_Update(MPU6050_Filter *f, MPU6050_ScaledData *data, float dt_s)
{
    // 由加速度计反算 Roll 和 Pitch (利用重力方向)
    // roll:  atan2(ay, az) → Y/Z方向的重力比
    // pitch: atan2(-ax, sqrt(ay²+az²)) → X与YZ平面的夹角
    float acc_roll  = atan2f(data->ay, data->az) * 57.29578f;      // 弧度转度
    float acc_pitch = atan2f(-data->ax, sqrtf(data->ay * data->ay + data->az * data->az)) * 57.29578f;

    if (f->last_time == 0) {
        // 第一帧: 直接用加速度计初始化角度 (避免从0开始缓慢收敛)
        f->angle.roll  = acc_roll;
        f->angle.pitch = acc_pitch;
        f->angle.yaw   = 0;
    } else {
        // 互补融合: 陀螺仪积分 + 加速度计修正
        f->angle.roll  = f->alpha * (f->angle.roll  + data->gx * dt_s) + (1.0f - f->alpha) * acc_roll;
        f->angle.pitch = f->alpha * (f->angle.pitch + data->gy * dt_s) + (1.0f - f->alpha) * acc_pitch;
        // 偏航无加速度计修正, 加死区防止静止时残余零偏漂移
        f->angle.yaw  += (fabsf(data->gz) < 0.15f) ? 0 : data->gz * dt_s;
    }
    f->last_time = 1;
}

/* ==================== 滑动平均滤波 (单轴去噪) ==================== */

void MPU6050_AvgFilter_Init(MPU6050_AvgFilter *f)
{
    f->idx = 0;
    f->count = 0;
    f->sum = 0;
    for (int i = 0; i < 20; i++) f->buf[i] = 0;
}

/**
 * 维护一个20点滑动窗口, 每个新值替换旧值, 返回窗口内平均值
 * 适合平滑加速度或陀螺仪某轴的原始数据
 */
float MPU6050_AvgFilter_Update(MPU6050_AvgFilter *f, float val)
{
    f->sum -= f->buf[f->idx];        // 减去即将被覆盖的旧值
    f->buf[f->idx] = val;            // 写入新值
    f->sum += val;                   // 累加新值
    f->idx++;
    if (f->idx >= 20) f->idx = 0;    // 环形缓冲区
    if (f->count < 20) f->count++;   // 前20次逐渐填满
    return f->sum / f->count;        // 返回当前窗口内的均值
}
