// 三电平逆变器延迟保护 — 故障消隐计时器 (ride-through blanking)
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/3_level_inv_delayed_protection_scheme
//   (type4_pwm_protection_source.h: DC 事件 + AQ 触发 + RED 上升沿延迟计数)
// 翻译为 HardC 纯C 版本 (TI 的 EPWM 硬件实现 → 软件计时器)
//
// 两层保护策略:
//   1. 主开关: 故障边沿立即逐周期关断 (TI: CBC 逐周期) — main_trip 镜像故障
//   2. 内开关: 故障保持 trip_delay 个周期后才关断 (TI: RED 上升沿延迟) —
//      ride-through 消隐, 短于 trip_delay 的瞬态被完全屏蔽
//   3. 恢复立即复位 (非锁存, 自重新布防): 故障清除 → 三者立即恢复, 计数清零
//
// 与 EPWM 寄存器解耦: 本组件只做计时/判定, trip 输出 (main_trip/inner_trip)
// 由应用/BSP 层接到 PWM 关断或急停。trip_delay=0 → 立即关断 (无消隐)。
//
// 每控制周期 (ISR) 调用 prot_3lvl_delay_tick() 一次; fault_asserted 来自
// 保护检测 (如 comp_protection.h 的去抖/阈值, 或逐周期比较器)

#ifndef COMP_PROTECTION_3LVL_H
#define COMP_PROTECTION_3LVL_H

#include <stdint.h>

// ======================= Prot3LvlDelay (延迟关断消隐计时器) =======================

typedef struct {
  uint32_t trip_delay;  // 参数: 延迟关断周期数 (消隐时间 = trip_delay·T)
  uint32_t blank_ctr;   // 内部: 故障消隐计数
  int      main_trip;   // 输出: 主开关立即关断 (逐周期, 镜像故障)
  int      inner_trip;  // 输出: 内开关延迟关断 (消隐后生效)
  int      active;      // 输出: 延迟关断当前已生效
} Prot3LvlDelay;

// 初始化
//   trip_delay — 延迟关断周期数 (0 = 立即关断)
static inline void prot_3lvl_delay_init(Prot3LvlDelay *me, uint32_t trip_delay) {
  me->trip_delay = trip_delay;
  me->blank_ctr = 0u;
  me->main_trip = 0;
  me->inner_trip = 0;
  me->active = 0;
}

// 逐周期推进 — 每控制周期调用一次
//   fault_asserted — 1 = 故障有效 (保护检测输出), 0 = 正常/已恢复
static inline void prot_3lvl_delay_tick(Prot3LvlDelay *me, int fault_asserted) {
  if (fault_asserted) {
    me->main_trip = 1;  // 主开关立即关断
    if (me->blank_ctr < me->trip_delay) {
      me->blank_ctr++;
    }
    if (me->blank_ctr >= me->trip_delay) {  // 消隐期满 → 内开关关断
      me->inner_trip = 1;
      me->active = 1;
    }
  } else {
    me->main_trip = 0;  // 恢复立即复位
    me->inner_trip = 0;
    me->active = 0;
    me->blank_ctr = 0u;  // 自重新布防
  }
}

#endif  // COMP_PROTECTION_3LVL_H
