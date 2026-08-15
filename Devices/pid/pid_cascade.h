// 角度+速度双环级联 PID —— 组合模块, 不继承 PidBase
//
// 架构: 组合优于继承 — 级联本质是两个独立 PID 控制器的串联编排:
//   外环(角度/位置) PID → 输出 / scale → 速度域 → 限幅 → 内环(速度) PID 目标
//   内环(速度) PID → 误差 = 速度目标 - 速度反馈 → 最终 PWM 输出
//
// 每个子 PID 独立管理自己的状态、限幅和抗积分饱和
// 级联模块只负责信号流编排: 串联 → 缩放 → 目标限幅

#ifndef PID_CASCADE_H
#define PID_CASCADE_H

#include "comp_pid.h"

typedef struct {
  PidBase *outer;         // 外环 PID (角度/位置, 通常为 PidStandard 或 PidP2PD)
  PidBase *inner;         // 内环 PID (速度, 通常为 PidStandard)
  float    scale;         // 角度输出 → 速度参考 缩放因子 (如 64)
  float    inner_limit;   // 内环目标限幅 (防止超速), 如 ±10
} PidCascade;

// ======== 构造 ========

// 初始化级联: 绑定内外环句柄 + 设置缩放/限幅
// outer/inner 必须提前由 pid_std_init / pid_p2pd_init 等初始化好
void pid_cascade_init(PidCascade *me, PidBase *outer, PidBase *inner,
                      float scale, float inner_limit);

// ======== 对外接口 (独立 API, 不通过 PidBase) ========

// 单次控制周期: 传入 角度目标 + 角度反馈 + 速度反馈, 返回最终 PWM
// outer_fb = 角度/位置测量值 (外环反馈)
// inner_fb = 速度测量值        (内环反馈)
float pid_cascade_compute(PidCascade *me, float target,
                          float outer_fb, float inner_fb);

// 清零内外环状态 (积分器 + 历史)
void pid_cascade_reset(PidCascade *me);

#endif
