#ifndef __PID_H
#define __PID_H

/**
 * ==================== 通用PID算法库 ====================
 *
 * 包含以下变体, 按需调用:
 *   PID_Compute()        → 位置式PID (基础)
 *   PID_ComputeInc()     → 增量式PID (返回Δu, 适用于步进/舵机)
 *   PID_ComputeDF()      → 微分先行 (目标突变不产生微分冲击)
 *   PID_ComputeFF()      → 前馈PID (已知目标变化时可提前补偿)
 *   PID_ComputeCascade() → 串级PID (两个PID首尾相接)
 *
 * 使用示例:
 *   PID_Controller pid;
 *   PID_Init(&pid, 1.0, 0.1, 0.05, 100, -100);
 *   float output = PID_Compute(&pid, target, actual, dt);
 * ===================================================================
 */

#include "main.h"

/* ==================== PID控制器 ==================== */
typedef struct {
    float kp, ki, kd;            // 比例/积分/微分系数
    float setpoint;              // 目标值 (串级PID外环用)
    float integral;              // 积分累积量
    float prev_error;            // 上一次误差
    float prev_measurement;      // 上一次测量值 (微分先行用)
    float prev_output;           // 上一次输出 (增量式用)
    float out_max, out_min;      // 输出限幅
    float integral_max;          // 积分限幅 (抗积分饱和)
} PID_Controller;

/* ==================== 初始化 ==================== */

/**
 * @brief 初始化PID控制器
 * @param out_max 输出上限 (如PWM=999)
 * @param out_min 输出下限 (如PWM=-999)
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float out_max, float out_min);

/**
 * @brief 运行时修改PID参数
 */
void PID_Tune(PID_Controller *pid, float kp, float ki, float kd);

/**
 * @brief 清零积分和历史误差 (切换目标/模式时调用)
 */
void PID_Reset(PID_Controller *pid);

/* ==================== 基础位置式PID ==================== */

/**
 * @brief  位置式PID, 返回绝对输出值
 * @param  actual 当前实际值
 * @param  target 目标值
 * @param  dt_s   时间间隔(秒)
 * @return 控制输出
 *
 * 公式: u = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 * 适用于: 电机速度控制、温度控制、一般闭环
 */
float PID_Compute(PID_Controller *pid, float target, float actual, float dt_s);

/* ==================== 增量式PID ==================== */

/**
 * @brief  增量式PID, 返回本次输出的增量 Δu
 * @return 增量值 (需外部累加到上次输出: u += Δu)
 *
 * 公式: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 * 优点: 不累加积分→不怕积分饱和; 误动作影响小; 切换模式无冲击
 * 适用于: 步进电机、舵机、位置保持
 */
float PID_ComputeInc(PID_Controller *pid, float target, float actual);

/* ==================== 微分先行PID ==================== */

/**
 * @brief  微分先行 (Derivative on Measurement)
 * @param  dt_s 时间间隔(秒)
 *
 * 区别: 只对测量值微分, 不对目标值微分
 * 公式: u = Kp*e + Ki*∫e*dt - Kd*d(measurement)/dt
 * 优点: 目标突变时不产生微分冲击
 * 适用于: 云台跟随、机械臂轨迹
 */
float PID_ComputeDF(PID_Controller *pid, float target, float actual, float dt_s);

/* ==================== 前馈PID ==================== */

/**
 * @brief  前馈PID: 基础PID + 预估补偿量
 * @param  feedforward 前馈量 (如目标速度×系数)
 *
 * 公式: u = PID输出 + feedforward
 * 优点: 已知目标变化趋势时提前加力, 减小跟踪误差
 * 适用于: 轨迹跟踪、云台随动
 */
float PID_ComputeFF(PID_Controller *pid, float target, float actual, float dt_s, float feedforward);

/* ==================== 串级PID ==================== */

/**
 * @brief  串级PID: 外环输出作为内环目标
 * @param  outer     外环PID (如角度环)
 * @param  inner     内环PID (如速度环)
 * @param  outer_target  外环目标 (如目标角度)
 * @param  outer_actual  外环反馈 (如当前角度)
 * @param  inner_actual  内环反馈 (如当前速度)
 * @param  dt_s     时间间隔
 * @return 内环PID输出 (最终PWM)
 *
 * 流程: 外环算角度误差→输出目标速度→内环算速度误差→输出PWM
 * 适用于: 平衡小车、倒立摆
 */
float PID_ComputeCascade(PID_Controller *outer, PID_Controller *inner,
                         float outer_target, float outer_actual,
                         float inner_actual, float dt_s);

#endif
