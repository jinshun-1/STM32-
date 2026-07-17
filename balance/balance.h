#ifndef __BALANCE_H
#define __BALANCE_H

#include "main.h"

/*
 * 直立摆状态反馈控制器
 * 状态: 摆角、摆角速度、小车位置、小车速度
 * 注意: 参数是安全初值，必须在摆杆被手扶在直立点附近时小功率试车。
 */

#define BALANCE_PERIOD_MS          5U
#define BALANCE_DT                 0.005f

#define BALANCE_PWM_MAX            80.0f
#define BALANCE_ANGLE_ARM_LIMIT     70.0f   /* 手扶进入该范围才允许启动 */
#define BALANCE_ANGLE_TRIP_LIMIT   260.0f   /* 倾倒保护 */
#define BALANCE_POSITION_LIMIT    6000.0f   /* 轨道/活动范围保护，单位: 编码器计数 */

/* 初始反馈增益；ANGLE/MOTOR方向可用 K_*_SIGN 修正 */
/*
 *  【调参说明】
 *    K_ANGLE      : 主力扶正。松手秒倒→加、猛甩→减    (典型范围 0.20~0.60)
 *    K_ANGLE_RATE : 阻尼消抖。高频抖→加、迟钝→减      (典型范围 0.01~0.06)
 *    K_POSITION   : 定住小车。漂移跑远→加              (典型范围 0.003~0.020)
 *    K_VELOCITY   : 减速稳车。来回晃→加                (典型范围 0.02~0.08)
 *
 *  【调参顺序】 K_ANGLE → K_ANGLE_RATE → K_VELOCITY → K_POSITION
 *    每次只改一个, 每次变动不超过20%
 *    前两项先调, 能短暂直立再加后两项
 */
#define BALANCE_K_ANGLE             0.2f    /* ① 角度增益  */
#define BALANCE_K_ANGLE_RATE        0.01f   /* ② 角速度增益 */
#define BALANCE_K_POSITION          0.01f   /* ③ 位置增益  */
#define BALANCE_K_VELOCITY          0.03f   /* ④ 速度增益  */

#define BALANCE_ANGLE_SIGN          1.0f
#define BALANCE_MOTOR_SIGN          1.0f

/* 启动时自动取 64 次均值作为直立零点 */
#define BALANCE_CALIBRATION_SAMPLES 64U

typedef enum
{
    BALANCE_STOPPED = 0,
    BALANCE_CALIBRATING,
    BALANCE_RUNNING,
    BALANCE_FAULT_ANGLE,
    BALANCE_FAULT_POSITION
} Balance_State;

void Balance_Init(void);
void Balance_CalibrateUpright(void);
uint8_t Balance_Start(void);
void Balance_Stop(void);
void Balance_ControlStep(int16_t encoder_delta);

Balance_State Balance_GetState(void);
int16_t Balance_GetAngle(void);
float Balance_GetAngleRate(void);
float Balance_GetCartSpeed(void);
float Balance_GetOutput(void);
int16_t Balance_GetZeroRaw(void);

#endif
