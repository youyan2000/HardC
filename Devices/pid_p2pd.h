#ifndef PID_P2PD_H
#define PID_P2PD_H

// P2PD 非线性 PID 控制器 —— PidBase 的子类
//
// 内模原理: 不含积分器 → 跟踪阶跃信号有静差, 但平方项提供自动增益调度
//   增益调度: |Err|小 (直线) → err²≈0, Kp 主导温和修正, 不会 Z 字振荡
//             |Err|大 (弯道) → err² 急剧增长, 平方项主导强力修正
//   无积分器 → 不需要抗积分饱和 (on_saturation = NULL)
//
// 适用场景: 循迹差速 (输入 pos -7~+7) / 转弯角度 (kpp=0 保持线性)

#include "comp_pid.h"

// ======== 配置结构体 ========
typedef struct {
  float kpp, kp, kd;   // 平方项 / 比例 / 微分系数
} PidP2PDConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase       base;          // 基类 (必须第一个)
  PidP2PDConfig cfg;           // 配置
  float         prev_error;    // 运行时状态: 上一拍误差 (D 项用)
} PidP2PD;

// ======== 构造 ========

// 初始化 P2PD: 调基类构造 → 复制配置 → 绑定 ops
void pid_p2pd_init(PidP2PD *me, float dt, float out_min, float out_max,
                   const PidP2PDConfig *cfg);

// ======== 运行时调参 ========

void pid_p2pd_update_config(PidP2PD *me, const PidP2PDConfig *cfg);
void pid_p2pd_set_kpp(PidP2PD *me, float kpp);
void pid_p2pd_set_kp(PidP2PD *me, float kp);
void pid_p2pd_set_kd(PidP2PD *me, float kd);

// 手动写入 prev_error: 用于转弯后切回循迹时抑制 D 尖峰
// 设 prev_error = 当前误差 → Δerr = 0 → D 项 = 0, 防止大 D 导致飞车
void pid_p2pd_set_prev_error(PidP2PD *me, float val);

#endif
