// BSP PWM 硬件抽象接口 —— Components/Devices 与硬件寄存器之间的边界
//
// 此接口由硬件后端实现:
//   STM32: bsp_hrtim.c (HRTIM on G4/H7/F3, ~184ps per tick)
//   C2000: bsp_c2000_epwm.c (ePWM on TMS320F28xxx, ~150ps per tick)
//
// Devices 层只调用此接口声明的函数, 不直接操作寄存器.
// BSP 层负责将逻辑参数 (频率/占空比/死区/相位) 翻译为具体寄存器写入.
//
// API 分为两层:
//   上层 API (推荐): 接收物理参数 (duty/Hz/ns/deg), BSP 内部换算为寄存器值
//   下层 API (保留): 接收寄存器值 (CMP/tick), 供需要精细控制的拓扑使用

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>
#include <stdbool.h>

// ======== 硬件句柄 (不透明指针, 平台无关) ========
typedef void BspPwmHandle;

// ======== 定时器索引 (平台无关, 最多 6 个独立定时器) ========
typedef enum {
  BSP_TIMER_A = 0,   // HRTIM Timer A / C2000 ePWM1
  BSP_TIMER_B,       // HRTIM Timer B / C2000 ePWM2
  BSP_TIMER_C,       // HRTIM Timer C / C2000 ePWM3
  BSP_TIMER_D,       // HRTIM Timer D / C2000 ePWM4
  BSP_TIMER_E,       // HRTIM Timer E / C2000 ePWM5
  BSP_TIMER_F,       // HRTIM Timer F / C2000 ePWM6
  BSP_TIMER_COUNT
} BspPwmTimer;

// ======== 输出通道掩码 ========
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

// ======== BSP 顶层配置 (生命周期) ========
typedef struct {
  BspPwmHandle *handle;       // 硬件句柄 (如 &hhrtim1, 由 App 注入)
  uint32_t      clk_hz;       // 基准时钟频率 (Hz), 如 STM32G4 DLL×32 = 5440000000
                              //   STM32F334 HCLK = 72000000
                              //   C2000 SYSCLK = 200000000
  bool          use_dll;      // [HRTIM 专属] true=启用 DLL 倍频提升死区分辨率
                              //   C2000 后端忽略此字段, 永远传 false
} BspPwmConfig;

// ======== 定时器物理参数配置 (上层 API, 推荐) ========
typedef enum {
  BSP_PWM_EDGE_ALIGNED,     // 边沿对齐 (简单 PWM, 单 CMP)
  BSP_PWM_CENTER_ALIGNED,   // 中心对齐 (对称 PWM, EMI 更低, 需双 CMP)
} BspPwmAlignMode;

typedef struct {
  BspPwmTimer     timer;            // 定时器编号
  uint32_t        freq_hz;          // 开关频率 (Hz)
  float           duty;             // 占空比 [0.0, 1.0]
  BspPwmAlignMode align;            // 对齐方式
  uint32_t        deadtime_ns;      // 死区时间 (纳秒), 仅互补输出有效
  bool            complementary;    // 是否互补输出 + 死区插入
  uint32_t        output_mask;      // 此通道对应的输出掩码
  float           phase_deg;        // 相位偏移 (度), 仅全桥/交错拓扑有效
} BspPwmChCfg;

// ======== 定时器寄存器级配置 (下层 API, 保留) ========
// 当上层 API 不够灵活时 (如非标准 PWM 调制), 可直接传寄存器值
typedef struct {
  BspPwmTimer timer;             // 使用的定时器编号
  uint32_t    period;            // PWM 周期 (计数值)
  uint32_t    cmp1;              // 比较值 1 (上升沿 / 简单 PWM 的 CCR)
  uint32_t    cmp2;              // 比较值 2 (中心参考 / 移相偏移)
  uint32_t    cmp3;              // 比较值 3 (下降沿 / 中心对齐第二沿)
  uint32_t    deadtime_rising;   // 上升沿死区 (BSP 内部 tick)
  uint32_t    deadtime_falling;  // 下降沿死区 (BSP 内部 tick)
  uint32_t    output_mask;       // 此定时器使能的输出通道掩码
  bool        complementary;     // true=互补输出 + 死区插入使能
} BspPwmTimerConfig;

// =====================================================================
//  上层 API (推荐): 传物理参数, BSP 内部换算
// =====================================================================

// 配置单个定时器 — 所有参数为物理量
void bsp_pwm_config_ch(BspPwmHandle *h, const BspPwmChCfg *cfg);

// 设置占空比 (热路径, ISR 安全)
void bsp_pwm_set_duty_f(BspPwmHandle *h, BspPwmTimer timer, float duty);

// 设置开关频率 — 重配时基, 需等待当前周期结束
void bsp_pwm_set_freq_hz(BspPwmHandle *h, BspPwmTimer timer, uint32_t freq_hz);

// 设置死区时间 (纳秒)
void bsp_pwm_set_deadtime_ns(BspPwmHandle *h, BspPwmTimer timer, uint32_t ns);

// 设置相位偏移 (度, 0~360)
void bsp_pwm_set_phase_deg(BspPwmHandle *h, BspPwmTimer timer, float phase_deg);

// =====================================================================
//  下层 API (保留): 传寄存器值, 用于非标准调制 / 向后兼容
// =====================================================================

// 初始化 BSP 实例 (时钟启用、GPIO 复用、DLL 校准)
// config.clk_hz 由调用方填充; handle 由 App 层注入
void bsp_init(BspPwmConfig *cfg);

// 配置单个定时器 (寄存器级)
void bsp_config_timer(BspPwmHandle *h, const BspPwmTimerConfig *tcfg);

// 启动 PWM: 先启动所有定时器计数器, 等波形对齐后使能输出
void bsp_start(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask);

// 停止 PWM: 先封输出, 再停计数器 (防止关断瞬间电平不确定)
void bsp_stop(BspPwmHandle *h, uint32_t timer_mask, uint32_t output_mask);

// 更新占空比 (热路径, 寄存器级 — 直接写 CMP1/CMP3)
void bsp_update_duty(BspPwmHandle *h, BspPwmTimer timer,
                     uint32_t cmp1, uint32_t cmp3);

// 更新周期 (变频用, 寄存器级)
void bsp_update_period(BspPwmHandle *h, BspPwmTimer timer, uint32_t period);

// 更新死区 (寄存器级, BSP 内部 tick)
void bsp_update_deadtime(BspPwmHandle *h, BspPwmTimer timer,
                         uint32_t rising_tick, uint32_t falling_tick);

// 紧急停机 (直接写故障寄存器, 不经过软件判断)
void bsp_emergency_stop(BspPwmHandle *h, uint32_t output_mask);

// =====================================================================
//  通用操作 (两层共用)
// =====================================================================

// 设置相位偏移 (全桥移相: 调整 B腿相对于 A腿的 CMP2 偏移)
void bsp_set_phase_shift(BspPwmHandle *h, BspPwmTimer timer, float phase_deg);

// 开关互补输出 — ISR 安全 (轻量寄存器操作)
// enable=true:  恢复互补输出 (同步整流)
// enable=false: 关闭互补输出 (二极管仿真 DEM, 阻断反灌)
void bsp_pwm_set_complementary(BspPwmHandle *h, BspPwmTimer timer, bool enable);

// BSP 级 ISR 入口: 由平台 ISR 调用, 内部处理后端特定事务
// 如: STM32 → HAL_HRTIM_MasterCallback() 中调用
//      C2000  → ePWM ISR 中调用
void bsp_pwm_isr(BspPwmHandle *h);

#endif  // BSP_PWM_H
