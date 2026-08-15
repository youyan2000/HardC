// 比例+微分 (PD) 控制器 —— PidBase 的子类
//
// 比例项 + 微分项: Out = Kp×(Ref−Fbk) + Kd×d(err)/dt
// 与 MATLAB PID 模块中"只用 P+D 项"等价
//
// 应用场景: 需要阻尼/预测特性的环路 (如速度环加阻尼、角度环抑制超调)、
//   位置跟踪系统的 PD 控制
//
// 微分路径:
//   - 默认微分作用在误差差商: (err − err_prev) / dt
//   - 可配置 d_on_measurement: true → 仅对测量值求导 (微分先行), 避免设定点跳变
//     引起微分冲击. 注意与误差同方向需取负: D = −Kd×(meas−meas_prev)/dt
//
// 抗噪声: 真实系统中微分项会放大噪声, 通常配合低通滤波 (本类不内置,
//   需要时可在 D 通路串 comp_filter).
//
// 首拍处理: 首次调用 / reset 后一个采样周期内微分项为 0 (尚无上一拍样本,
//   避免微分冲击), 之后正常差分.

#ifndef PID_PD_H
#define PID_PD_H

#include "comp_pid.h"
#include <stdbool.h>

// ======== 配置结构体 (与运行时状态分离) ========
typedef struct {
  float kp;               // 比例增益
  float kd;               // 微分增益
  bool  d_on_measurement; // true=微分先行(只对测量值求导), false=标准D(对误差求导)
} PidPdConfig;

// ======== 子类结构体 —— 基类必须为第一成员 ========
typedef struct {
  PidBase base;           // 基类 (必须为第一成员, container_of 依赖)
  PidPdConfig cfg;        // 可热替换的配置

  // 状态 (上一拍的误差/测量值, 用于差分)
  float err_prev;         // 上拍误差
  float meas_prev;        // 上拍测量值 (微分先行时用)
  bool  initialized;      // 是否已有上一拍样本 (首拍微分项置 0, 避免微分冲击)
} PidPd;

// 构造 (自动绑定 ops + 基类)
void pid_pd_init(PidPd *me, float dt, float out_min, float out_max,
                 const PidPdConfig *cfg);

// 运行时替换配置
void pid_pd_update_config(PidPd *me, const PidPdConfig *cfg);

// 逐个参数热修改
void pid_pd_set_kp(PidPd *me, float kp);
void pid_pd_set_kd(PidPd *me, float kd);

#endif  // PID_PD_H
