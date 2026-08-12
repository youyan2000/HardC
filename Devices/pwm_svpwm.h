// 六开关 SVPWM —— PwmBase 子类, 三相电压源逆变器空间矢量调制
//
// 来源: TI controlSUITE motor_control/math_blocks/v4.3 (SVGEN)
// 翻译为 C-OOP 风格
//
// 拓扑: 3 个半桥 (6 开关) 构成三相逆变桥
// 应用: 三相 AC/DC 整流器、三相 DC/AC 并网逆变器、PMSM/BLDC 电机驱动
//
// 调制方法:
//   7 段式 (对称): V0(000)→V1→V2→V7(111)→V2→V1→V0, 零矢量在两端和中间
//   5 段式 (断续): 每 60° 一段不切换, 减少 1/3 开关损耗
//
// 接口:
//   svpwm_set_vector(&svpwm, v_alpha, v_beta, v_dc_bus);  // 设置电压矢量
//
// 调用链 (典型 ISR):
//   1. 反Park 变换: dq → αβ (comp_transform.h park_inv_run)
//   2. SVPWM: αβ → 6 路占空比 (本文件)
//   3. BSP: 6 路占空比 → 硬件比较寄存器 (BSP/bsp_pwm.h)

#ifndef PWM_SVPWM_H
#define PWM_SVPWM_H

#include "comp_pwm.h"
#include "bsp_pwm.h"

// SVPWM 调制模式
typedef enum {
  SvpwmMode_7Seg,       // 7 段式对称调制 (默认, 正弦输出, THD 最优)
  SvpwmMode_5Seg,       // 5 段式不连续调制 (降开关损耗, 效率优先)
} SvpwmMode;

// DPWM 不连续调制子模式 (仅 SvpwmMode_5Seg 时生效)
// 来源: TI controlSUITE svgen_dpwm.h
typedef enum {
  SvpwmDpwm_MIN,   // DPWMMIN: 最低相钳位到 0, 仅用零矢量 V0
  SvpwmDpwm_MAX,   // DPWMMAX: 最高相钳位到 1, 仅用零矢量 V7
  SvpwmDpwm_0,     // DPWM0: 60°交替, 奇扇区→MIN(0), 偶扇区→MAX(1)
  SvpwmDpwm_1,     // DPWM1: 60°交替(反向), 奇扇区→MAX(1), 偶扇区→MIN(0)
  SvpwmDpwm_2,     // DPWM2: 30°钳位, 中间相<0.5→MIN, >0.5→MAX
} SvpwmDpwmMode;

// 六开关 SVPWM 子类
typedef struct {
  PwmBase base;                       // 基类 (必须为第一个成员)

  // BSP 硬件绑定 — 3 个半桥腿 = 6 个开关管
  BspPwmHandle *bsph;                // BSP 硬件句柄 (board_init 绑定, NULL=未绑定)
  BspPwmTimer   timer_a;             // A 相定时器
  BspPwmTimer   timer_b;             // B 相定时器
  BspPwmTimer   timer_c;             // C 相定时器
  uint32_t      output_mask_a;       // A 相输出掩码
  uint32_t      output_mask_b;       // B 相输出掩码
  uint32_t      output_mask_c;       // C 相输出掩码

  // 输入 — 电压矢量 (αβ 坐标系)
  float  v_alpha;                     // α 轴电压 (标幺值, -1~+1, 相对 0.5*Vdc)
  float  v_beta;                      // β 轴电压 (标幺值, -1~+1)
  float  v_dc_bus;                    // 直流母线电压 (V), 用于标幺→绝对占空比换算

  // 输出 — 三相占空比 [0, 1]
  float  duty_a;                      // A 相上管占空比
  float  duty_b;                      // B 相上管占空比
  float  duty_c;                      // C 相上管占空比

  // 内部 — 扇区与矢量时间
  uint8_t sector;                     // 当前扇区 (1~6)
  SvpwmMode mode;                     // 调制模式
  float  t1;                          // 第一有效矢量作用时间 (标幺)
  float  t2;                          // 第二有效矢量作用时间 (标幺)

  // 配置
  float    overmod_limit;             // 过调制阈值 (默认 1.0, >1 进入过调制)
  bool     overmod_enable;            // 是否允许过调制 (默认关闭, 输出纯正弦)
  SvpwmDpwmMode dpwm_mode;        // DPWM 子模式选择 (默认 DPWMMIN)
} PwmSvpwm;

// 构造 — 绑定 3 路半桥定时器
//   freq_hz: 开关频率 (如 20000 = 20kHz)
//   deadtime_ns: 死区 (ns)
//   timer_a/b/c: 三相定时器
//   output_mask_a/b/c: 每相输出通道掩码
void svpwm_init(PwmSvpwm *me, uint32_t freq_hz, uint32_t deadtime_ns,
                BspPwmTimer timer_a, uint32_t output_mask_a,
                BspPwmTimer timer_b, uint32_t output_mask_b,
                BspPwmTimer timer_c, uint32_t output_mask_c);

// 反初始化
void svpwm_deinit(PwmSvpwm *me);

// 设置 αβ 电压矢量 — 自动计算扇区 + 6 路占空比
//   v_alpha, v_beta: αβ 轴电压 (标幺值, -0.577~+0.577 为线性调制区)
//   v_dc_bus: 当前直流母线电压 (用于标幺→伏秒换算)
void svpwm_set_vector(PwmSvpwm *me, float v_alpha, float v_beta, float v_dc_bus);

// 设置调制模式
void svpwm_set_mode(PwmSvpwm *me, SvpwmMode mode);

// 设置 DPWM 子模式 (仅 5Seg 模式下生效)
void svpwm_set_dpwm_mode(PwmSvpwm *me, SvpwmDpwmMode dpwm_mode);

// 计算当前调制比 (0~1, 1=线性调制边界, >1=过调制)
float svpwm_get_modulation_index(const PwmSvpwm *me);

#endif  // PWM_SVPWM_H
