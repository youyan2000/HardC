// PID_GRANDO 全功能 PID 控制器 — 设定点权重 + D 滤波 + 前馈 + 回算抗饱和
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (pid_grando.h)
// 翻译为 C-OOP PidBase 子类
//
// 与 PidDcl 的关键差异:
//   1. Km 前馈权重 — 加 D 项可对设定点 Kg 加权 (缺省 Km=0)
//   2. 回算法抗饱和 — 通过 w1 标记, 在 compute 中禁止积分而非乘法冻结
//   3. Kp 作用于三路之和 — up/ui/ud/Kp 顺序与 DCL 不同
//
// 算法:
//   up = Kr * Ref - Fbk                               (设定点权重)
//   ud = c1 * Kd * (Km*Ref - Fbk) - d2                (D 路径一阶滤波)
//   d2 = ud * c2                                       (滤波反馈)
//   ui = ui_prev + Ki * w1 * (Ref - Fbk)              (回算抗积分)
//   Out = clamp(Kp * (up + ui + ud), Umin, Umax)      (Kp 在外)

#ifndef PID_GRANDO_H
#define PID_GRANDO_H

#include "comp_pid.h"

// ======== 配置结构体 ========
typedef struct {
  float kr;               // 设定点权重 (0=I-PD, 1=标准PID)
  float kp;               // 比例增益 (作用于三路和)
  float ki;               // 积分增益
  float kd;               // 微分增益
  float km;               // D 项设定点权重 (0=微分只对反馈, 1=微分也对设定点)
  float c1;               // D 滤波器输入系数
  float c2;               // D 滤波器反馈系数
  float out_max;          // 输出上限
  float out_min;          // 输出下限
} PidGrandoConfig;

// ======== 子类结构体 ========
typedef struct {
  PidBase base;                    // 基类 (必须为第一个成员)
  PidGrandoConfig cfg;

  // 状态
  float up;                        // 比例项
  float ui;                        // 积分项
  float ui_prev;                   // 上拍积分
  float ud;                        // 微分项 (滤波后)
  float d2;                        // D 滤波器内部状态
  float w1;                        // 饱和度标记 (1.0=未饱和, 0.0=饱和)
} PidGrando;

// 构造
void pid_grando_init(PidGrando *me, float dt,
                     const PidGrandoConfig *cfg);

// 运行时替换配置
void pid_grando_update_config(PidGrando *me, const PidGrandoConfig *cfg);

// 逐个参数热修改
void pid_grando_set_kp(PidGrando *me, float kp);
void pid_grando_set_ki(PidGrando *me, float ki);
void pid_grando_set_kd(PidGrando *me, float kd);
void pid_grando_set_kr(PidGrando *me, float kr);

// 设置 D 项滤波器截止频率
void pid_grando_set_dfilt_freq(PidGrando *me, float fc_hz);

#endif  // PID_GRANDO_H
