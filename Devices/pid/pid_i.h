// 纯积分 (I) 控制器 —— PidBase 的子类
//
// 仅积分项: Out += Ki × dt × (Ref − Fbk) ，消除稳态误差的唯一手段
// 与 MATLAB PID 模块中"只用 I 项"等价
//
// 适用场景: 需要彻底消除稳态误差但允许较慢响应的环路、
//           与其它控制器串联时的"输出调节"、需要对阶跃/恒值渐近跟踪的场合
//
// 抗积分饱和: 采用 clamping(截幅) 抗饱和 —— 输出被基类限幅截断时回调把积分器钳到
//   限幅边界 (本类输出==积分累积值), 不 windup. 若将来叠加 P 项须重写抗饱和(见 .c).

#ifndef PID_I_H
#define PID_I_H

#include "comp_pid.h"

// ======== 配置结构体 (与运行时状态分离) ========
typedef struct {
  float ki;               // 积分增益
} PidIConfig;

// ======== 子类结构体 —— 基类必须为第一成员 ========
typedef struct {
  PidBase base;           // 基类 (必须为第一成员, container_of 依赖)
  PidIConfig cfg;         // 可热替换的配置

  // 状态
  float integral;         // 积分累积值
} PidI;

// 构造 (自动绑定 ops + 基类)
void pid_i_init(PidI *me, float dt, float out_min, float out_max,
                const PidIConfig *cfg);

// 运行时替换配置
void pid_i_update_config(PidI *me, const PidIConfig *cfg);

// 逐个参数热修改
void pid_i_set_ki(PidI *me, float ki);

#endif  // PID_I_H
