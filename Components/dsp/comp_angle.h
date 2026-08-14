// 电机控制 — 角度归算宏 (标幺 0~1 范围处理)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (angle_math.h)
// 翻译为 HardC 纯C float inline 版本
//
// 两种归算:
//   angle_wrap — 角度归算到 [0.0, 1.0) (相位折叠)
//   error_angle_wrap — 角度误差归算到 [-0.5, +0.5) (最小相位差)
//
// 用于: 电机矢量控制的角度积分、锁相环的相位折叠

#ifndef COMP_ANGLE_H
#define COMP_ANGLE_H

#include "comp_math.h"

// ======================= ANGLE_WRAP (角度归算 0~1) =======================

// 将角度折叠到 [0.0, 1.0) 范围 (标幺)
// if(angle > 1.0) angle -= 1.0
// if(angle < 0.0) angle += 1.0
static inline float angle_wrap(float angle) {
  if (angle > 1.0f) {
    angle -= 1.0f;
  } else if (angle < 0.0f) {
    angle += 1.0f;
  }
  return angle;
}

// ======================= ERROR_ANGLE_WRAP (误差归算 -0.5~+0.5) =======================

// 将角度误差折叠到 [-0.5, +0.5) 范围 (最短路径)
// 用于角度差计算 (如 PLL 鉴相、位置环误差)
// 例如: error = target_angle - current_angle, 然后 error_angle_wrap(error)
static inline float error_angle_wrap(float angle_error) {
  if (angle_error > 0.5f) {
    angle_error -= 1.0f;
  } else if (angle_error < -0.5f) {
    angle_error += 1.0f;
  }
  return angle_error;
}

// ======================= ANGLE_WRAP_2PI (弧度版 0~2π) =======================

// 将弧度角度折叠到 [0, 2π)
static inline float angle_wrap_2pi(float angle_rad) {
  while (angle_rad > M_2PI) {
    angle_rad -= M_2PI;
  }
  while (angle_rad < 0.0f) {
    angle_rad += M_2PI;
  }
  return angle_rad;
}

// 将弧度误差归算到 [-π, +π)
static inline float error_angle_wrap_2pi(float error_rad) {
  while (error_rad > M_PI) {
    error_rad -= M_2PI;
  }
  while (error_rad < -M_PI) {
    error_rad += M_2PI;
  }
  return error_rad;
}

#endif  // COMP_ANGLE_H
