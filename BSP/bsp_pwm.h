#ifndef BSP_PWM_H
#define BSP_PWM_H

// BSP PWM 硬件抽象接口 —— Components/Devices 与硬件寄存器之间的边界
//
// 此接口由两种硬件后端实现:
//   bsp_hrtim.c      — STM32 HRTIM (G4/H7/F3 系列)
//   bsp_c2000_epwm.c — TI C2000 ePWM (TMS320F28xxx)
//
// Devices 层只调用此接口声明的函数, 不直接操作寄存器.
// BSP 层负责将逻辑参数 (频率/占空比/死区/相位) 翻译为具体寄存器写入.

#include <stdint.h>
#include <stdbool.h>

// ======== 硬件句柄 (不透明指针) ========
typedef void BspPwmHandle;

// ======== 定时器索引 ========
typedef enum {
  BSP_TIMER_A = 0,   // HRTIM Timer A / C2000 ePWM1
  BSP_TIMER_B,       // HRTIM Timer B / C2000 ePWM2
  BSP_TIMER_C,       // HRTIM Timer C / C2000 ePWM3
  BSP_TIMER_D,       // HRTIM Timer D / C2000 ePWM4
  BSP_TIMER_E,       // HRTIM Timer E / C2000 ePWM5
  BSP_TIMER_F,       // HRTIM Timer F / C2000 ePWM6
  BSP_TIMER_COUNT
} BspPwmTimer;

// ======== 输出通道掩码 (每个定时器有 2 路输出) ========
#define BSP_OUT_TA1  (1u << 0)    // Timer A 输出 1 (高侧)
#define BSP_OUT_TA2  (1u << 1)    // Timer A 输出 2 (低侧 / 互补)
#define BSP_OUT_TB1  (1u << 2)    // Timer B 输出 1
#define BSP_OUT_TB2  (1u << 3)    // Timer B 输出 2
#define BSP_OUT_TC1  (1u << 4)    // Timer C 输出 1
#define BSP_OUT_TC2  (1u << 5)    // Timer C 输出 2
#define BSP_OUT_TD1  (1u << 6)    // Timer D 输出 1
#define BSP_OUT_TD2  (1u << 7)    // Timer D 输出 2
#define BSP_OUT_TE1  (1u << 8)    // Timer E 输出 1
#define BSP_OUT_TE2  (1u << 9)    // Timer E 输出 2
#define BSP_OUT_TF1  (1u << 10)   // Timer F 输出 1
#define BSP_OUT_TF2  (1u << 11)   // Timer F 输出 2

// 常用组合掩码
#define BSP_OUT_TIMER_A_PAIR  (BSP_OUT_TA1 | BSP_OUT_TA2)
#define BSP_OUT_TIMER_B_PAIR  (BSP_OUT_TB1 | BSP_OUT_TB2)
#define BSP_OUT_TIMER_C_PAIR  (BSP_OUT_TC1 | BSP_OUT_TC2)
#define BSP_OUT_TIMER_D_PAIR  (BSP_OUT_TD1 | BSP_OUT_TD2)
#define BSP_OUT_TIMER_E_PAIR  (BSP_OUT_TE1 | BSP_OUT_TE2)
#define BSP_OUT_TIMER_F_PAIR  (BSP_OUT_TF1 | BSP_OUT_TF2)

// ======== BSP 配置结构体 ========
typedef struct {
  BspPwmHandle *handle;          // 硬件句柄 (如 &hhrtim1)
  uint32_t      hrtim_clk_hz;    // HRTIM 基准时钟 (如 STM32G4: 5440000000 = 5.44GHz for DLL x32)
  bool          use_dll;         // true=使用 DLL 倍频 (HRTIM), false=直接时钟分频 (ePWM)
} BspPwmConfig;

// ======== 定时器独立配置 ========
typedef struct {
  BspPwmTimer timer;             // 使用的定时器编号
  uint32_t    period;            // PWM 周期 (计数值, BSP 内部根据频率计算)
  uint32_t    cmp1;              // 比较值 1 (上升沿 / 简单 PWM 的 CCR)
  uint32_t    cmp2;              // 比较值 2 (中心点参考)
  uint32_t    cmp3;              // 比较值 3 (下降沿)
  uint32_t    deadtime_rising;   // 上升沿死区 (BSP tick)
  uint32_t    deadtime_falling;  // 下降沿死区 (BSP tick)
  uint32_t    output_mask;       // 此定时器使能的输出通道掩码
  bool        complementary;     // true=互补输出 + 死区插入使能
} BspPwmTimerConfig;

// ======== BSP 函数声明 ========

// 初始化 BSP PWM 实例 (时钟使能、GPIO 复用、DLL 校准)
// config 由调用方填充 clk/dll 字段, BSP 内部填充 handle
void bsp_init(BspPwmConfig *cfg);

// 配置单个定时器
void bsp_config_timer(BspPwmHandle *h, const BspPwmTimerConfig *tcfg);

// 启动 PWM: 先启动所有定时器计数器, 等波形对齐后使能输出
void bsp_start(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask);

// 停止 PWM: 先封输出, 再停计数器
void bsp_stop(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask);

// 更新占空比 (热路径 —— 直接写比较寄存器, 不经过 HAL)
void bsp_update_duty(BspPwmHandle *h, BspPwmTimer timer,
                          uint32_t cmp1, uint32_t cmp3);

// 更新周期 (变频用, 需等待当前周期结束生效)
void bsp_update_period(BspPwmHandle *h, BspPwmTimer timer, uint32_t period);

// 更新死区
void bsp_update_deadtime(BspPwmHandle *h, BspPwmTimer timer,
                              uint32_t rising_tick, uint32_t falling_tick);

// 紧急停机 (直接写 ODISR / TZFRC 等故障寄存器, 不经过软件判断)
void bsp_emergency_stop(BspPwmHandle *h, uint32_t output_mask);

// 设置相位偏移 (全桥移相: 调整 B腿相对于 A腿的 CMP2 偏移)
void bsp_set_phase_shift(BspPwmHandle *h, BspPwmTimer timer, float phase_deg);

#endif
