// 电机控制 — 旋转变压器接口 (Resolver)
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (resolver.h)
// 翻译为 C-OOP 纯C float inline 版本
//
// 旋变: 绝对位置传感器, 通过解码 sin/cos 励磁信号获取机械角度
//   电角度 = 极对数 × 机械角度
//   机械角度 = (RawPosition - Offset) × Scaler
//
// 定标: 输入 RawTheta 为 Q0 整数 (0~StepsPerTurn), 输出为弧度

#ifndef COMP_RESOLVER_H
#define COMP_RESOLVER_H

#include <math.h>
#include <stdint.h>

#ifndef M_2PI
#define M_2PI 6.283185f
#endif

// ======================= Resolver (旋转变压器解码) =======================

typedef struct {
  // 输入
  float raw_theta;        // 输入: 原始位置 (整数计数, 0~StepsPerTurn)

  // 输出
  float elec_theta;       // 输出: 电角度 (rad)
  float mech_theta;       // 输出: 机械角度 (rad)
  float speed;            // 输出: 转速 (标幺)

  // 参数
  float init_theta;       // 参数: 初始角度偏移 (Q0)
  float mech_scaler;      // 参数: 机械角度缩放 = 2π/总计数
  float steps_per_turn;   // 参数: 每圈离散位置数
  uint16_t pole_pairs;    // 参数: 极对数
} Resolver;

#define RESOLVER_DEFAULTS { 0, 0, 0, 0, 0, 0.0001f, 4096, 2 }

// 初始化
//   steps_per_turn: 每圈编码器位置数 (如 4096)
//   pole_pairs:     电机极对数 (如 2=4极, 4=8极)
static inline void resolver_init(Resolver *me, float steps_per_turn,
                                  uint16_t pole_pairs) {
  me->raw_theta = 0.0f;
  me->elec_theta = 0.0f;
  me->mech_theta = 0.0f;
  me->speed = 0.0f;
  me->init_theta = 0.0f;
  me->mech_scaler = M_2PI / steps_per_turn;
  me->steps_per_turn = steps_per_turn;
  me->pole_pairs = pole_pairs;
}

// 旋变解码单步运行
//   raw_position: 当前原始编码器读数 (0 ~ StepsPerTurn-1)
//   返回: 电角度 (rad)
static inline float resolver_decode(Resolver *me, float raw_position) {
  me->raw_theta = raw_position;

  // 机械角度 = (原始值 - 偏移) × 缩放因子
  me->mech_theta = (raw_position - me->init_theta) * me->mech_scaler;

  // 电角度 = 极对数 × 机械角度
  me->elec_theta = (float)me->pole_pairs * me->mech_theta;

  // 角度折叠 — 保留分数部分
  while (me->elec_theta > M_2PI) {
    me->elec_theta -= M_2PI;
  }
  while (me->elec_theta < 0.0f) {
    me->elec_theta += M_2PI;
  }

  return me->elec_theta;
}

// 设置机械角度偏移 (校准用)
static inline void resolver_set_offset(Resolver *me, float offset) {
  me->init_theta = offset;
}

#endif  // COMP_RESOLVER_H
