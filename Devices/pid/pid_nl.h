// 非线性 PID 控制器 —— PidBase 子类 (TI C2000Ware DCL NLPID)
//
// 与线性 PID 的区别: 三个通路 (P/I/D) 各自带非线性整形函数
//   f(e) = sign(e)·|e|^α      (|e| > δ, 幂律区)
//   f(e) = e·γ                 (|e| ≤ δ, 线性区, γ 为小误差增益)
// 用途: 小误差高增益 / 大误差降增益的强鲁棒控制 (电源启动、负载突变)
//
// 来源: TI C2000Ware Digital Power SDK c2000ware/libraries/control/DCL/c28/include/DCL_NLPID.h
//       翻译为 HardC PidBase 子类 (算法来自 comp_pid_nl.h, 包装为子类形态)
//
// 并行式结构: 积分带抗饱和 (i16 标志), 微分带二阶滤波 (c1/c2)
// 输入/输出归一化 ±1 (pu), 误差在 run 内折半预处理 (与 DCL 一致)

#ifndef PID_NL_H
#define PID_NL_H

#include "comp_pid.h"
#include "comp_math.h"

// ======================= 配置 POD =======================
typedef struct {
  float kp, ki, kd;           // 线性增益
  float alpha_p, alpha_i, alpha_d;  // 幂律指数 (0<α<2)
  float delta_p, delta_i, delta_d;  // 线性化范围
  float gamma_p, gamma_i, gamma_d;  // 线性区增益
  float c1, c2;               // D 通路滤波器系数
} PidNlConfig;

// ======================= 子类结构体 —— 基类第一成员 =======================
typedef struct {
  PidBase  base;
  PidNlConfig cfg;
  // 运行时状态
  float d2, d3;               // D 通路滤波器中间状态
  float i7;                   // I 通路积分
  float i16;                  // 抗饱和标志 (1=未饱和, 0=钳位停积分)
} PidNl;

// 构造 (自动绑定 ops + 基类)
void pid_nl_init(PidNl *me, float dt, float out_min, float out_max,
                 const PidNlConfig *cfg);
// 运行时调参
void pid_nl_update_config(PidNl *me, const PidNlConfig *cfg);
void pid_nl_set_kp(PidNl *me, float v);
void pid_nl_set_ki(PidNl *me, float v);
void pid_nl_set_kd(PidNl *me, float v);
void pid_nl_set_filter_bw(PidNl *me, float fc, float dt);   // 微分滤波带宽
void pid_nl_set_gamma_from_delta(PidNl *me);

#endif  // PID_NL_H
