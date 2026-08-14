// SFRA 软件频率响应分析仪 — 在线 Bode 图测量
//
// 来源: TI controlSUITE SFRA/v1.20/Float (SFRA_F_Include.h)
// 翻译为 HardC 纯C float 版本
//
// 应用: 在线测量电源/电机控制环路频率响应, 无需外接 FRA 仪器
//
// 算法:
//   1. 正弦扰动注入 (DDS) — 在控制环路参考点叠加小信号
//   2. DFT/Goertzel 提取 — 注入点和响应点做激励频率的 DFT
//   3. 增益/相位 — 比较注入和响应的幅值比 + 相位差
//   4. 对数扫频 — f_start → f_end, 每频点停留 settle+measure 周期
//
// 典型 ISR 调用:
//   mod_sfra_inject(&sfra);
//   ref += sfra.inject_out;      // 扰动叠加到参考点
//   ... 控制算法 ...
//   mod_sfra_collect(&sfra, output);
//
// 主循环:
//   mod_sfra_background(&sfra);   // 频点切换 + DFT 结算 + 结果输出

#ifndef MOD_SFRA_H
#define MOD_SFRA_H

#include <stdbool.h>
#include <stdint.h>

// 用户配置 — 扫频参数
typedef struct {
  float f_start;            // 起始频率 (Hz)
  float f_end;              // 终止频率 (Hz)
  float inject_amp;         // 扰动幅值 (标幺, 典型 0.02~0.05)
  int   points_per_decade;  // 每十倍频点数 (典型 10~20)
  int   settle_cycles;      // 频点切换后稳定周期数 (典型 10~50)
  int   measure_cycles;     // 每频点测量周期数 (典型 50~200)
} SfraCfg;

// SFRA 运行时状态 — 栈/静态分配, 零 malloc
typedef struct {
  SfraCfg cfg;              // 用户配置 (不变)

  // 扫频状态
  float current_freq;       // 当前注入频率 (Hz)
  float current_omega;      // 当前角频率 (rad/s)
  int   freq_index;         // 当前频点序号 (0-based)
  int   total_freqs;        // 总频点数
  int   cycle_counter;      // 当前频点已过 ISR 周期数
  bool  settling;           // true=稳定中, false=测量中
  bool  sweep_done;         // 扫频完成
  bool  running;            // 扫频进行中

  // DDS 正弦注入 (连续运行, 跨频点不重置相位)
  float inject_phase;       // 当前 DDS 相位 (rad), 连续累加
  float inject_out;         // 本次扰动值 inject_amp * sin(inject_phase)

  // DFT 参考相位 (每频点测量窗口从 0 开始)
  float dft_phase;          // DFT 参考相位 (rad)

  // DFT 累加器 (注入通道)
  float inj_re;             // Σ inject[n] * cos(ωnT)
  float inj_im;             // Σ inject[n] * (-sin)(ωnT)

  // DFT 累加器 (响应通道)
  float resp_re;            // Σ response[n] * cos(ωnT)
  float resp_im;            // Σ response[n] * (-sin)(ωnT)

  // 当前频点结果
  float gain_db;            // 增益 (dB) = 20*log10(|H|)
  float phase_deg;          // 相位 (deg), 归一化到 [-180, 180]

  int   sample_count;       // 当前频点已采点数
  float dt;                 // ISR 周期 (s)

  // 用户回调: 每频点完成后调用, 用于串口/GUI 发送结果
  void (*on_point_done)(void *user, int idx, int total,
                         float freq, float gain_db, float phase_deg);
  void *user_data;          // 回调上下文
} Sfra;

// ======== 初始化 & 重置 ========

// 初始化: 绑定配置和采样周期, 清零所有运行时状态
void mod_sfra_init(Sfra *me, float dt, const SfraCfg *cfg);

// 重置: 回到空闲状态, 保留 cfg/dt/回调
void mod_sfra_reset(Sfra *me);

// ======== 扫频控制 ========

// 启动扫频: 计算频点列表 → 从 f_start 开始 → settling
void mod_sfra_start(Sfra *me);

// 中止扫频: 立即停止, sweep_done=false
void mod_sfra_stop(Sfra *me);

// ======== ISR 调用 (快速路径, 禁止 printf) ========

// 扰动注入 — 在控制算法之前调用, 输出注入值到 me->inject_out
// 用户将 me->inject_out 叠加到控制环参考点
void mod_sfra_inject(Sfra *me);

// 响应采集 — 在控制算法之后调用
//   response: 被测输出变量 (如 Vout, Iout, 编码器速度等)
void mod_sfra_collect(Sfra *me, float response);

// ======== 主循环调用 (慢速路径, 可 printf) ========

// 后台处理 — 频点切换、DFT 结算、回调通知
void mod_sfra_background(Sfra *me);

#endif  // MOD_SFRA_H
