// 电力测量 — 真有效值/有功/无功/视在功率/功率因数 + 能量积分
//
// 来源: TI C2000Ware Digital Power SDK
//   libraries/power_measurement/include/power_meas_sine_analyzer.h
//   libraries/energy-metrology_library/energy_metrology_f28p55/
//     (metrology_background.c 逐采样累加, metrology_calculations.c 读数计算,
//      energyIntegrator 能量脉冲积分)
// 翻译为 C-OOP 纯C float 版本 (合并两库的功率测量算法精华)
//
// 单相功率计量核心:
//   1. 逐采样累加 (ISR 热路径): v²/ i²/ (v·i)/ (v_quad·i)
//   2. 无功: 正交电压 v_quad = 电压历史环形缓冲延迟 1/4 周期 + 插值 → 90° 移相
//      Q = Σ(v_quad·i)/N  (与 IEC 62053 单相表计同一方法)
//   3. 每窗口结算 (每 N 个采样调用一次):
//      Vrms = √(Σv²/N)   Irms = √(Σi²/N)   P = Σ(v·i)/N
//      S = hypot(P, Q)   PF = P/S   φ = atan2(Q, P)
//   4. 能量积分: energy += P·Δt, 每积累满阈值 (如 0.1 Wh) 输出一个脉冲
//      残余结转下一拍 (防丢失精度), 导入/导出分开累计
//   5. DC 去除: 一阶高通 y += (x−y)/16384, 输出 x−y (消除 ADC 直流偏置)
//
// 三相聚合: 文件尾部 PowerMeas3Ph 容器 — 每相内嵌 PowerMeas 独立测量,
// 聚合总 P/Q/S + 线电压 + 电流矢量和 + 聚合功率因数 (见该节说明)
//
// 输入电压/电流需先归一化到标幺 (pu), 或由用户自行换算标度 (scale 由外部处理)

#ifndef COMP_POWER_MEAS_H
#define COMP_POWER_MEAS_H

#include <math.h>
#include <stdint.h>

// 电压历史环形缓冲大小 (2 的幂, 掩码索引)
#define POWER_MEAS_V_HISTORY_SIZE  1024u
#define POWER_MEAS_V_HISTORY_MASK  (POWER_MEAS_V_HISTORY_SIZE - 1u)

// ======================= DC 去除滤波器 (一阶高通) =======================
//
// y += (x − y)·(1/16384) ; 输出 xfilt = x − y
// 极低频截止 (~Fs/10000), 只滤 ADC 直流偏置, 不伤 50/60Hz 基波

static inline float power_meas_dc_filter(float *state, float x) {
  const float coeff = 1.0f / 16384.0f;
  *state += (x - *state) * coeff;
  return x - *state;
}

// ======================= PowerMeas (单相功率测量) =======================

typedef struct {
  // 输入
  float v_sample;         // 输入: 当前电压采样 (去 DC 后)
  float i_sample;         // 输入: 当前电流采样 (去 DC 后)

  // 参数
  float sample_rate;      // 参数: 采样频率 (Hz)
  float threshold;        // 参数: 过零检测阈值
  uint16_t window_n;      // 参数: 结算窗口采样数 (= 一个周期 N, 如 10kHz/50Hz → 200)
  uint16_t quad_delay;    // 参数: 正交延迟采样数 (= N/4, 90° 移相)

  // DC 滤波器状态
  float v_dc_state;       // 电压 DC 滤波状态
  float i_dc_state;       // 电流 DC 滤波状态

  // 电压历史环形缓冲 (正交延迟用)
  float v_history[POWER_MEAS_V_HISTORY_SIZE];
  uint16_t v_history_idx;

  // 每窗口累加器
  float v_sq_sum;         // 累加: Σv²
  float i_sq_sum;         // 累加: Σi²
  float p_sum;            // 累加: Σ(v·i)  — 有功
  float q_sum;            // 累加: Σ(v_quad·i) — 无功
  uint32_t sample_count;  // 累加: 窗口内采样数

  // 输出 (每窗口更新)
  float vrms;             // 输出: 电压真有效值
  float irms;             // 输出: 电流真有效值
  float active_power;     // 输出: 有功功率 P
  float reactive_power;   // 输出: 无功功率 Q
  float apparent_power;   // 输出: 视在功率 S = hypot(P,Q)
  float power_factor;     // 输出: 功率因数 PF = P/S (钳位到 [-1,1])
  float pf_angle;         // 输出: 功率因数角 φ = atan2(Q,P) (rad)

  // 频率测量 (过零法)
  float freq;             // 输出: 信号频率 (Hz)
  int   zcd;              // 输出: 过零标志 (1=正向过零)
  int   prev_sign;        // 上一拍符号 (0=负, 1=正)
  int   curr_sign;        // 当前符号
  uint32_t period_count;  // 周期采样计数 (相邻正向过零之间, 少算过零样本自身)
} PowerMeas;

// 初始化 — 每窗口 N 个采样结算一次功率读数
//   sample_rate — 采样频率 Hz
//   threshold   — 过零阈值
//   window_n    — 结算窗口采样数 (建议 = 一个周期采样数)
static inline void power_meas_init(PowerMeas *me, float sample_rate,
                                   float threshold, uint16_t window_n) {
  me->sample_rate = sample_rate;
  me->threshold = threshold;
  me->window_n = window_n;
  me->quad_delay = (uint16_t)(window_n / 4u);   // 90° 移相 = 1/4 周期

  me->v_dc_state = 0.0f;
  me->i_dc_state = 0.0f;
  me->v_history_idx = 0;

  me->v_sq_sum = 0.0f;
  me->i_sq_sum = 0.0f;
  me->p_sum = 0.0f;
  me->q_sum = 0.0f;
  me->sample_count = 0;

  me->vrms = 0.0f;
  me->irms = 0.0f;
  me->active_power = 0.0f;
  me->reactive_power = 0.0f;
  me->apparent_power = 0.0f;
  me->power_factor = 0.0f;
  me->pf_angle = 0.0f;

  me->freq = 0.0f;
  me->zcd = 0;
  me->prev_sign = 1;
  me->curr_sign = 1;
  me->period_count = 0;
}

// 单采样累加 — ISR 热路径, 每采样周期调用一次
//   v/i — 当前电压/电流采样 (推荐已过外部标度换算)
static inline void power_meas_sample(PowerMeas *me, float v, float i) {
  // 1. DC 去除 (ADC 直流偏置 → 0)
  v = power_meas_dc_filter(&me->v_dc_state, v);
  i = power_meas_dc_filter(&me->i_dc_state, i);
  me->v_sample = v;
  me->i_sample = i;

  // 2. 累加 v² / i² / (v·i)
  me->v_sq_sum += v * v;
  me->i_sq_sum += i * i;
  me->p_sum += v * i;

  // 3. 正交电压: 历史延迟 1/4 周期 + 相邻样本插值 → 90° 移相
  //    v_quad = v_history[idx−delay−1]·β + v_history[idx−delay]
  //    (β=0.5: 两样本中点, 对应 90° 附近相位)
  const uint16_t idx_prev = (uint16_t)(me->v_history_idx - me->quad_delay - 1u)
                            & POWER_MEAS_V_HISTORY_MASK;
  const uint16_t idx_next = (uint16_t)(me->v_history_idx - me->quad_delay)
                            & POWER_MEAS_V_HISTORY_MASK;
  const float v_quad = me->v_history[idx_prev] * 0.5f + me->v_history[idx_next];

  me->q_sum += i * v_quad;

  // 4. 写回电压历史
  me->v_history[me->v_history_idx] = v;
  me->v_history_idx = (uint16_t)(me->v_history_idx + 1u)
                      & POWER_MEAS_V_HISTORY_MASK;

  me->sample_count++;

  // 5. 过零检测 (频率测量)
  if (v > me->threshold) {
    me->curr_sign = 1;
  } else {
    me->curr_sign = 0;
  }

  if ((me->prev_sign != me->curr_sign) && (me->curr_sign == 1)) {
    me->zcd = 1;
    // 相邻正向过零 = 一个整周期; period_count 少算过零样本自身 → +1
    if (me->period_count > 0) {
      me->freq = me->sample_rate / ((float)me->period_count + 1.0f);
    }
    me->period_count = 0;
  } else {
    me->zcd = 0;
    me->period_count++;
  }
  me->prev_sign = me->curr_sign;
}

// 窗口结算 — 每 N 个采样调用一次, 计算 Vrms/Irms/P/Q/S/PF/φ 并清累加器
static inline void power_meas_update(PowerMeas *me) {
  if (me->sample_count == 0u) {
    return;
  }

  const float inv_n = 1.0f / (float)me->sample_count;

  me->vrms = sqrtf(me->v_sq_sum * inv_n);
  me->irms = sqrtf(me->i_sq_sum * inv_n);
  me->active_power = me->p_sum * inv_n;
  me->reactive_power = me->q_sum * inv_n;
  me->apparent_power = hypotf(me->active_power, me->reactive_power);

  if (me->apparent_power > 0.0f) {
    float pf = me->active_power / me->apparent_power;
    if (pf > 1.0f) pf = 1.0f;
    else if (pf < -1.0f) pf = -1.0f;
    me->power_factor = pf;
  } else {
    me->power_factor = 0.0f;
  }

  me->pf_angle = atan2f(me->reactive_power, me->active_power);

  me->v_sq_sum = 0.0f;
  me->i_sq_sum = 0.0f;
  me->p_sum = 0.0f;
  me->q_sum = 0.0f;
  me->sample_count = 0;
}

// 重置状态 (保留配置)
static inline void power_meas_reset(PowerMeas *me) {
  float sr = me->sample_rate;
  float th = me->threshold;
  uint16_t wn = me->window_n;
  power_meas_init(me, sr, th, wn);
}

// ======================= EnergyAccu (能量积分器) =======================
//
// 将功率积分成能量, 每积累满阈值输出一个脉冲 (模拟表计 LED/刻度盘)。
// 亚阈值残余结转下一拍, 避免截断丢失精度。

typedef struct {
  float energy;           // 输出: 累计能量 (Wh)
  float residual;         // 内部: 亚阈值残余 (Wh, 结转下一拍)
  float threshold;        // 参数: 脉冲阈值 (Wh, 如 0.1)
  uint32_t pulses;        // 输出: 累计脉冲计数
  float energy_import;    // 输出: 累计吸收能量 (Wh, P>0)
  float energy_export;    // 输出: 累计馈出能量 (Wh, P<0)
} EnergyAccu;

// 初始化
//   threshold_wh — 脉冲阈值 (Wh), 如 0.1 → 每 0.1Wh 一个脉冲
static inline void energy_accu_init(EnergyAccu *me, float threshold_wh) {
  me->energy = 0.0f;
  me->residual = 0.0f;
  me->threshold = threshold_wh;
  me->pulses = 0u;
  me->energy_import = 0.0f;
  me->energy_export = 0.0f;
}

// 积分一步 — 每结算窗口调用一次
//   power_w — 本窗口平均功率 (W, 允许带符号)
//   dt_s    — 窗口时长 (s) = N / sample_rate
//   返回: 本次产生的脉冲数
static inline uint32_t energy_accu_integrate(EnergyAccu *me, float power_w,
                                             float dt_s) {
  // 与 TI 一致: 先取绝对值再积分 (方向由导入/导出路由决定)
  const float abs_power = fabsf(power_w);

  // 功率×时间 → 能量 (W·s = J), 折算 Wh (1 Wh = 3600 J), 加残余
  float energy_wh = abs_power * dt_s / 3600.0f + me->residual;

  uint32_t steps = 0u;
  while (energy_wh >= me->threshold) {
    energy_wh -= me->threshold;
    steps++;
  }

  me->residual = energy_wh;
  me->energy += (float)steps * me->threshold;
  me->pulses += steps;

  // 导入/导出分离累计 (符号按功率方向)
  if (power_w >= 0.0f) {
    me->energy_import += (float)steps * me->threshold;
  } else {
    me->energy_export += (float)steps * me->threshold;
  }

  return steps;
}

// ======================= PowerMeas3Ph (三相功率聚合) =======================
//
// 三相仅聚合: 每相独立用 PowerMeas 单相测量, 本容器做总功率/线电压/
// 电流矢量和/聚合功率因数
// 来源: TI metrology_calculations.c calculateTotalActivePower/ReactivePower/
//   ApparentPower + calculateLinetoLineVoltage + calculateVectorCurrentSum +
//   calculateAggregatePowerfactor (energy-metrology_library)
//
// 总 P/Q/S = 三相标量和, 残差 < 0.5 截断为 0 (TOTAL_RESIDUAL_POWER_CUTOFF)
// 线电压 (相量差): Vll = hypot(V1 − V2·cos(θ), V2·sin(θ)), θ = 相位差 rad
//   平衡 120° → Vll = √3·V 相电压
// 电流矢量和 (TI): θ1 = (φab + PFA0 − PFA1)·0.5, θ2 = (φbc + PFA1 − PFA2)·0.5
//   ix = I0 + I1·cos(θ1·2π) + I2·cos((θ2+θ1)·2π)
//   iy = I1·sin(θ1·2π) + I2·sin((θ2+θ1)·2π)   结果 = hypot(ix, iy)
//   平衡三相电流矢量相互抵消 → ≈0; 不平衡时反映矢量和
// 聚合 PF = 总P/总S, 总P<0 时取反 (发电机惯例)
//
// 相位差 per-unit: 1.0 = 180° (TI phasetoPhaseAngle 约定, 平衡 120° → 2/3).
//   默认平衡 120°, 并网时用 set_phase_deg 喂 SrfPll 锁相角

typedef struct {
  // 参数
  float sample_rate;          // 采样频率 (Hz)
  float threshold;            // 过零阈值 (透传各相)
  uint16_t window_n;          // 结算窗口采样数 (透传各相)
  float phase_pu[3];          // 相邻相位差 per-unit (1.0 = 180°): ab/bc/ca

  // 子测量
  PowerMeas phases[3];        // 每相单相测量

  // 输出 (update 后有效)
  float total_p;              // 总有功 (W)
  float total_q;              // 总无功 (VAR)
  float total_s;              // 总视在 (VA)
  float aggregate_pf;         // 聚合功率因数 (0~1)
  float ll_v[3];              // 线电压 Vab/Vbc/Vca (V)
  float vector_i_sum;         // 电流矢量和 (A)
  float freq;                 // 三相频率 (取 A 相, Hz)
} PowerMeas3Ph;

// 设置相邻相位差 (度) — 默认平衡 120/120/120, 并网时喂锁相角
static inline void power_meas_3ph_set_phase_deg(PowerMeas3Ph *me,
                                                float ab_deg, float bc_deg,
                                                float ca_deg) {
  me->phase_pu[0] = ab_deg / 180.0f;
  me->phase_pu[1] = bc_deg / 180.0f;
  me->phase_pu[2] = ca_deg / 180.0f;
}

// 初始化 — 3 个单相 PowerMeas + 聚合输出清零
static inline void power_meas_3ph_init(PowerMeas3Ph *me, float sample_rate,
                                       float threshold, uint16_t window_n) {
  me->sample_rate = sample_rate;
  me->threshold = threshold;
  me->window_n = window_n;
  power_meas_3ph_set_phase_deg(me, 120.0f, 120.0f, 120.0f);

  for (int i = 0; i < 3; i++) {
    power_meas_init(&me->phases[i], sample_rate, threshold, window_n);
  }

  me->total_p = 0.0f;
  me->total_q = 0.0f;
  me->total_s = 0.0f;
  me->aggregate_pf = 0.0f;
  me->ll_v[0] = 0.0f;
  me->ll_v[1] = 0.0f;
  me->ll_v[2] = 0.0f;
  me->vector_i_sum = 0.0f;
  me->freq = 0.0f;
}

// 单采样累加 — ISR 热路径, 每采样周期调用一次 (三相同时采样)
static inline void power_meas_3ph_sample(PowerMeas3Ph *me, float va, float ia,
                                         float vb, float ib, float vc,
                                         float ic) {
  power_meas_sample(&me->phases[0], va, ia);
  power_meas_sample(&me->phases[1], vb, ib);
  power_meas_sample(&me->phases[2], vc, ic);
}

// 线电压 (相量差): Vll = hypot(V1 − V2·cos(θ), V2·sin(θ)), θ = pu·π
static inline float power_meas_3ph_ll(float v1, float v2, float pu_angle) {
  float th = pu_angle * 3.14159265359f;
  float x = v1 - v2 * cosf(th);
  float y = v2 * sinf(th);
  return hypotf(x, y);
}

// 电流矢量和 (TI calculateVectorCurrentSum) — 三相电流 + 相位差 + 功率因数角
static inline float power_meas_3ph_vector_sum(const PowerMeas3Ph *me) {
  const float pi = 3.14159265359f;
  // 功率因数角 per-unit (1.0 = 180°): pf_angle/π
  float pfa0 = me->phases[0].pf_angle / pi;
  float pfa1 = me->phases[1].pf_angle / pi;
  float pfa2 = me->phases[2].pf_angle / pi;

  // θ1/θ2 与 TI 一致: (相位差 + PFA 差)·0.5, 后乘 2π
  float t1 = (me->phase_pu[0] + pfa0 - pfa1) * 0.5f;
  float t2 = (me->phase_pu[1] + pfa1 - pfa2) * 0.5f;
  float a1 = t1 * 2.0f * pi;
  float a2 = (t2 + t1) * 2.0f * pi;

  float i0 = me->phases[0].irms;
  float i1 = me->phases[1].irms;
  float i2 = me->phases[2].irms;

  float ix = i0 + i1 * cosf(a1) + i2 * cosf(a2);
  float iy = i1 * sinf(a1) + i2 * sinf(a2);
  return hypotf(ix, iy);
}

// 窗口结算 — 结算 3 相 + 聚合: 总 P/Q/S, 聚合 PF, 线电压, 电流矢量和
static inline void power_meas_3ph_update(PowerMeas3Ph *me) {
  for (int i = 0; i < 3; i++) {
    power_meas_update(&me->phases[i]);
  }

  // 总 P/Q/S: 标量和 + 残差截断 (TOTAL_RESIDUAL_POWER_CUTOFF = 0.5)
  float sum_p = 0.0f, sum_q = 0.0f, sum_s = 0.0f;
  for (int i = 0; i < 3; i++) {
    sum_p += me->phases[i].active_power;
    sum_q += me->phases[i].reactive_power;
    sum_s += me->phases[i].apparent_power;
  }
  me->total_p = (fabsf(sum_p) < 0.5f) ? 0.0f : sum_p;
  me->total_q = (fabsf(sum_q) < 0.5f) ? 0.0f : sum_q;
  me->total_s = (fabsf(sum_s) < 0.5f) ? 0.0f : sum_s;

  // 聚合功率因数: |总P|/总S, 总P<0 取反 (发电机惯例)
  if (me->total_s > 0.0f) {
    float pf = me->total_p / me->total_s;
    if (pf > 1.0f) pf = 1.0f;
    else if (pf < -1.0f) pf = -1.0f;
    me->aggregate_pf = fabsf(pf);
  } else {
    me->aggregate_pf = 0.0f;
  }

  // 线电压 (相邻相位差): Vab/Vbc/Vca
  me->ll_v[0] = power_meas_3ph_ll(me->phases[0].vrms, me->phases[1].vrms,
                                  me->phase_pu[0]);
  me->ll_v[1] = power_meas_3ph_ll(me->phases[1].vrms, me->phases[2].vrms,
                                  me->phase_pu[1]);
  me->ll_v[2] = power_meas_3ph_ll(me->phases[2].vrms, me->phases[0].vrms,
                                  me->phase_pu[2]);

  // 电流矢量和 + 频率 (取 A 相)
  me->vector_i_sum = power_meas_3ph_vector_sum(me);
  me->freq = me->phases[0].freq;
}

#endif  // COMP_POWER_MEAS_H
