// 电机控制 — InstaSPIN-BLDC 无传感器方波驱动 (6步换向 + 反电动势过零点检测)
//
// 来源: TI InstaSPIN-BLDC 算法概念 (SPRA590/SPRA695/SPRABQ7), 解绑自 ROM 实现
// 翻译为 C-OOP 纯C float inline 版本
//
// 算法概述:
//   BLDC 电机采用 6 步梯形换向 (方波驱动), 每 60° 电角度换向一次。
//   任意时刻: 一相 PWM+ (上管调制), 一相 PWM- (下管导通), 一相悬空 (高阻态)。
//   悬空相的反电动势 (BEMF) 过零点标志着转子到达下一个换向位置 —
//   过零点后延迟 30° 电角度即为下一次换向时刻。
//
// 控制流程:
//   ALIGN (强制对齐) → OPENLOOP (开环加速) → CLOSED (闭环换向 + 速度PI)
//       ↓                      ↓                      ↓
//     FAULT ←———————————————— (堵转 / 过流) ——————————┘
//
// 调用方式 (每个 PWM/控制周期在 ISR 中调用):
//   bldc_instaspin_run(&st, &cfg, v_bus, v_a, v_b, v_c, i_dc, dt);
//   int step = bldc_instaspin_get_commutation_step(&st);   // 0~5, 查表设置 PWM
//   float duty = bldc_instaspin_get_duty(&st);             // 占空比 0.0~1.0
//   float speed = bldc_instaspin_get_speed(&st);            // 电频率 (Hz)
//   bldc_instaspin_get_phase_state(&st, &pa, &pb, &pc);   // 获取三相开关状态
//
// 设计要点:
//   - 全部 static inline, ISR 安全 (无堆分配, 无阻塞)
//   - 悬空相电压与 Vbus/2 (虚拟中性点) 比较检测过零点
//   - 符号变化 + 连续确认滤波器消除噪声误触发
//   - 换向后空白窗口 (blanking) 避免续流二极管振铃误触发
//   - 速度 PI 带抗积分饱和 (anti-windup)

#ifndef COMP_BLDC_INSTASPIN_H
#define COMP_BLDC_INSTASPIN_H

#include <math.h>
#include <stdbool.h>

// ======== 枚举: 电机运行状态 ========

// clang-tidy: performance-enum-size is informational, ok for embedded
typedef enum {
  BLDC_STATE_ALIGN,     // 强制对齐 — 两相通电锁定转子到已知位置
  BLDC_STATE_OPENLOOP,  // 开环加速 — 强制换向, 频率递增, 同时监测反电动势
  BLDC_STATE_CLOSED,    // 闭环运行 — 反电动势过零点触发换向 + 速度PI调节占空比
  BLDC_STATE_FAULT      // 故障停机 — 堵转或过流, 占空比清零
} BldcMotorState;

// ======== 枚举: 故障码 ========

// clang-tidy: performance-enum-size is informational, ok for embedded
typedef enum {
  BLDC_FAULT_NONE = 0,        // 无故障
  BLDC_FAULT_STALL = 1,       // 堵转: 连续 N 步未检测到过零点
  BLDC_FAULT_OVERCURRENT = 2  // 过流: 母线电流超过阈值
} BldcFaultCode;

// ======== 配置参数 POD (纯数据结构, YmaC 可注入) ========

typedef struct {
  int pole_pairs;                // 电机极对数 (例如 7 对极 = 14 极转子)
  float pwm_freq_hz;             // PWM 频率 (Hz), 典型 20kHz
  float min_speed_hz;            // 闭环切入最低电频率 (Hz), 低于此速度 BEMF 太弱不可靠
  float max_speed_hz;            // 最高电频率 (Hz), 用于占空比限幅参考
  float align_time_s;            // 对齐阶段持续时间 (s), 典型 0.3~1.0
  float align_duty;              // 对齐阶段占空比 (0.0~1.0), 典型 0.05~0.15
  float startup_accel_hz_per_s;  // 开环加速率 (Hz/s, 电频率), 典型 50~200
  float kp_speed;                // 速度环比例增益
  float ki_speed;                // 速度环积分增益
  float zc_blanking_us;          // 换向后过零检测空白窗口 (us), 典型 50~200
  int zc_filter_cnt;             // 过零点连续确认次数 (2~3), 滤除噪声
  float overcurrent_threshold;   // 过流阈值 (A), 母线电流超过此值触发 FAULT
  int stall_timeout_steps;       // 堵转判定: 连续多少换向步未检测到 ZC 即报堵转
} BldcInstaSpinCfg;

// ======== 运行时状态 (每个电机独立实例) ========

typedef struct {
  // ---- 换向 ----
  int commutation_step;   // 当前换向步 (0~5), 对应 6 步真值表

  // ---- 输出 ----
  float duty;             // PWM 占空比 (0.0~1.0), 由速度 PI 或开环/对齐设定
  float speed_est;        // 估算电频率 (Hz), 由过零点周期反算
  float speed_cmd;        // 目标电频率 (Hz), 外部写入

  // ---- 速度 PI 控制器 ----
  float pi_integral;      // 速度 PI 积分项, 带抗饱和

  // ---- 过零点 (ZC) 检测 ----
  bool zc_detected;       // 当前换向步是否已检测到有效过零点
  float zc_diff_prev;     // 上次采样的悬空相电压差值 (v_float - Vbus/2)
  int zc_confirm_cnt;     // 过零点连续确认计数器 (达到 zc_filter_cnt 才算有效)
  float zc_ts_prev;       // 倒数第二次过零点时间戳 (s), 用于计算过零周期
  float zc_ts_cur;        // 最近一次过零点时间戳 (s)
  float zc_period;        // 相邻两次过零点时间间隔 (s), 对应 60° 电角度

  // ---- 运行时计时 (以 dt 累积, 自 init 起) ----
  float timer;            // 全局累计时间 (s)
  float last_comm_ts;     // 上次换向发生时刻 (s)
  float zc_confirm_ts;    // 过零点确认时刻 (s), 用于 30° 延迟计时
  float stage_enter_ts;   // 进入当前运行阶段的时刻 (s)

  // ---- 开环加速 ----
  float openloop_speed;   // 开环当前指令电频率 (Hz), 从 0 递增至 min_speed_hz

  // ---- 状态 ----
  BldcMotorState motor_state;  // 当前运行状态
  BldcFaultCode fault_code;    // 故障码 (FAULT 状态下有效)
  int stall_counter;           // 堵转计数: 未检测到 ZC 的连续换向步数
} BldcInstaSpinState;

// ======== 内部辅助: 根据换向步和三相电压获取悬空相电压 ========

static inline float bldc_instaspin_floating_voltage(int step, float va, float vb, float vc) {
  switch (step) {
    case 0:  // A→B, 悬空 C
    case 3:  // B→A, 悬空 C
      return vc;
    case 1:  // A→C, 悬空 B
    case 4:  // C→A, 悬空 B
      return vb;
    case 2:  // B→C, 悬空 A
    case 5:  // C→B, 悬空 A
      return va;
    default:
      return 0.0f;
  }
}

// ======== 默认配置 — 典型 hobby BLDC (7 对极, 20kHz PWM) 的合理初值 ========

static inline void bldc_instaspin_cfg_default(BldcInstaSpinCfg *cfg) {
  cfg->pole_pairs = 7;
  cfg->pwm_freq_hz = 20000.0f;
  cfg->min_speed_hz = 5.0f;
  cfg->max_speed_hz = 200.0f;
  cfg->align_time_s = 0.5f;
  cfg->align_duty = 0.08f;
  cfg->startup_accel_hz_per_s = 100.0f;
  cfg->kp_speed = 0.01f;
  cfg->ki_speed = 0.1f;
  cfg->zc_blanking_us = 100.0f;
  cfg->zc_filter_cnt = 3;
  cfg->overcurrent_threshold = 5.0f;
  cfg->stall_timeout_steps = 100;
}

// ======== 初始化 — 清零所有状态, 进入 ALIGN 阶段 ========

static inline void bldc_instaspin_init(BldcInstaSpinState *me) {
  me->commutation_step = 0;
  me->duty = 0.0f;
  me->speed_est = 0.0f;
  me->speed_cmd = 0.0f;
  me->pi_integral = 0.0f;

  me->zc_detected = false;
  me->zc_diff_prev = 0.0f;
  me->zc_confirm_cnt = 0;
  me->zc_ts_prev = 0.0f;
  me->zc_ts_cur = 0.0f;
  me->zc_period = 0.0f;

  me->timer = 0.0f;
  me->last_comm_ts = 0.0f;
  me->zc_confirm_ts = 0.0f;
  me->stage_enter_ts = 0.0f;

  me->openloop_speed = 0.0f;

  me->motor_state = BLDC_STATE_ALIGN;
  me->fault_code = BLDC_FAULT_NONE;
  me->stall_counter = 0;
}

// ======== 主运行函数 — 每个 PWM/控制周期在 ISR 中调用一次 ========
//
// 参数:
//   st:    运行时状态指针
//   cfg:   配置参数 (只读)
//   v_bus: 直流母线电压 (V)
//   v_a:   A 相端电压 (V), 由电阻分压 + ADC 采样
//   v_b:   B 相端电压 (V)
//   v_c:   C 相端电压 (V)
//   i_dc:  直流母线电流 (A), 由分流电阻 + ADC 采样, 用于过流保护
//   dt:    本次调用距上次调用的时间间隔 (s), 通常 = 1/pwm_freq_hz
//
// 函数更新 st 内部状态, 不返回值。调用后通过 getter 获取换向步、占空比、速度。

static inline void bldc_instaspin_run(BldcInstaSpinState *st, const BldcInstaSpinCfg *cfg,
                                       float v_bus, float v_a, float v_b, float v_c,
                                       float i_dc, float dt) {
  // ---- 全局计时累加 ----
  st->timer += dt;

  // ---- 过流检测 (所有阶段均有效) ----
  if (i_dc > cfg->overcurrent_threshold) {
    st->motor_state = BLDC_STATE_FAULT;
    st->fault_code = BLDC_FAULT_OVERCURRENT;
    st->duty = 0.0f;
    return;
  }

  // ---- 如果已故障, 不再执行任何控制, 占空比保持 0 ----
  if (st->motor_state == BLDC_STATE_FAULT) {
    return;
  }

  // ---- 阶段0: ALIGN — 强制对齐转子到已知位置 ----
  // 通电 A→B (换向步0), 固定小占空比, 保持 align_time_s
  // 转子在电磁力作用下旋转并对齐到 A 相轴线
  if (st->motor_state == BLDC_STATE_ALIGN) {
    st->commutation_step = 0;  // A→B 通电
    st->duty = cfg->align_duty;

    if (st->timer - st->stage_enter_ts >= cfg->align_time_s) {
      // 对齐完成, 切换到开环加速阶段
      st->motor_state = BLDC_STATE_OPENLOOP;
      st->stage_enter_ts = st->timer;
      st->last_comm_ts = st->timer;
      st->openloop_speed = 0.0f;
    }
    return;
  }

  // ---- 阶段1: OPENLOOP — 开环强制换向加速 ----
  // 以固定加速度 ramp 电频率从 0 至 min_speed_hz
  // 强制换向, 不依赖反电动势; 同时监测过零点以判断锁相是否成功
  if (st->motor_state == BLDC_STATE_OPENLOOP) {
    // 计算当前开环速度 (线性 ramp)
    float elapsed = st->timer - st->stage_enter_ts;
    st->openloop_speed = cfg->startup_accel_hz_per_s * elapsed;
    if (st->openloop_speed > cfg->min_speed_hz) {
      st->openloop_speed = cfg->min_speed_hz;
    }

    // 计算开环换向周期
    // 6 步/电周期, 换向间隔 = 1 / (6 × 电频率)
    float comm_period;
    if (st->openloop_speed > 0.01f) {
      comm_period = 1.0f / (6.0f * st->openloop_speed);
    } else {
      comm_period = 1.0f;  // 速度极低时用长周期, 避免除零
    }

    // 固定占空比, 随速度线性增加 (V/f 控制近似)
    // duty = align_duty + (1.0 - align_duty) * (speed / min_speed) * 0.3
    // 留余量, 最高只用到 30% 左右的开环占空比
    float speed_ratio = st->openloop_speed / cfg->min_speed_hz;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    st->duty = cfg->align_duty + (1.0f - cfg->align_duty) * speed_ratio * 0.3f;

    // 强制换向: 当累计时间达到换向间隔, 前进到下一步
    if (st->timer - st->last_comm_ts >= comm_period) {
      st->commutation_step = (st->commutation_step + 1) % 6;
      st->last_comm_ts = st->timer;
      st->zc_detected = false;
      st->zc_confirm_cnt = 0;
    }

    // 开环期间监测过零点 (用于判断锁相, 但不用于换向)
    // 换向后等待空白窗口结束后才开始检测
    float blanking_s = cfg->zc_blanking_us * 1e-6f;
    if (st->timer - st->last_comm_ts >= blanking_s) {
      float v_float = bldc_instaspin_floating_voltage(st->commutation_step, v_a, v_b, v_c);
      float diff = v_float - v_bus * 0.5f;

      // 过零点检测: 监测悬空相电压穿越 Vbus/2 (虚拟中性点)
      // zc_confirm_cnt > 0 表示已检测到潜在过零, 正处于确认窗口
      // 确认窗口内持续采样, 确认信号确实越过了零点而非噪声抖动
      if (st->zc_confirm_cnt > 0) {
        // 确认窗口内: 检查信号是否回穿 (逆向穿越 = 噪声)
        if (st->zc_diff_prev * diff < 0.0f) {
          // 信号回穿 — 噪声干扰, 重置确认窗口
          st->zc_confirm_cnt = 0;
        } else {
          // 信号稳定在新的一侧, 累积确认计数
          st->zc_confirm_cnt++;
          if (st->zc_confirm_cnt >= cfg->zc_filter_cnt && !st->zc_detected) {
            // 过零点确认 — 记录时间戳
            st->zc_detected = true;
            st->zc_confirm_cnt = 0;
            st->zc_ts_prev = st->zc_ts_cur;
            st->zc_ts_cur = st->timer;
            if (st->zc_ts_prev > 0.0f) {
              st->zc_period = st->zc_ts_cur - st->zc_ts_prev;
            }
          }
        }
      } else if (!st->zc_detected && (st->zc_diff_prev * diff < 0.0f)) {
        // 首次检测到符号变化 → 进入确认窗口
        st->zc_confirm_cnt = 1;
      }
      st->zc_diff_prev = diff;
    }

    // 切换条件: 开环速度达到 min_speed_hz 且至少检测到一次过零点
    if (st->openloop_speed >= cfg->min_speed_hz && st->zc_ts_cur > 0.0f) {
      st->motor_state = BLDC_STATE_CLOSED;
      st->stage_enter_ts = st->timer;
      st->stall_counter = 0;

      // 初始化 PI 积分器为当前占空比 (平滑过渡, 避免速度跳变)
      st->pi_integral = st->duty;
      st->speed_est = st->openloop_speed;

      // 如果没有记录到第二个 ZC 时间戳 (只有一次过零), 则用开环周期估算
      if (st->zc_ts_prev <= 0.0f && st->openloop_speed > 0.01f) {
        st->zc_ts_prev = st->zc_ts_cur - 1.0f / (6.0f * st->openloop_speed);
        st->zc_period = st->zc_ts_cur - st->zc_ts_prev;
      }
    }
    return;
  }

  // ---- 阶段2: CLOSED — 闭环反电动势过零点换向 + 速度PI ----
  if (st->motor_state == BLDC_STATE_CLOSED) {
    float blanking_s = cfg->zc_blanking_us * 1e-6f;

    // ---- 空白窗口检查 ----
    // 换向后的一段时间内, 悬空相电压因续流二极管振铃而不稳定,
    // 必须跳过这段时间, 不做过零检测
    bool in_blanking = (st->timer - st->last_comm_ts) < blanking_s;

    // ---- 过零点检测 (空白窗口外) ----
    // 过零点检测采用 "确认窗口" 机制:
    //   1. 检测到符号变化 + 超过滞环阈值 → 进入确认窗口 (zc_confirm_cnt = 1)
    //   2. 确认窗口内: 每采样一次信号仍在新侧 → 计数 +1; 信号回穿 → 窗口重置
    //   3. 计数达到 zc_filter_cnt → 过零点确认, 记录时间戳, 更新速度
    // 该机制有效滤除换向振铃后的残余噪声和 ADC 采样抖动
    if (!in_blanking && !st->zc_detected) {
      float v_float = bldc_instaspin_floating_voltage(st->commutation_step, v_a, v_b, v_c);
      float diff = v_float - v_bus * 0.5f;

      // 滞环阈值: 要求信号幅度超过 Vbus 的 2% 才算有效穿越, 过滤小幅噪声
      float zc_threshold = v_bus * 0.02f;
      bool crossed = (st->zc_diff_prev * diff < 0.0f)
                  && (fabsf(st->zc_diff_prev) > zc_threshold
                      || fabsf(diff) > zc_threshold);

      if (st->zc_confirm_cnt > 0) {
        // 确认窗口内: 检查信号是否稳定在新的一侧
        if (crossed) {
          // 信号回穿 (逆向穿越) — 噪声干扰, 重置确认窗口
          st->zc_confirm_cnt = 0;
        } else {
          // 信号稳定, 累积确认计数
          st->zc_confirm_cnt++;
          if (st->zc_confirm_cnt >= cfg->zc_filter_cnt) {
            // ---- 过零点确认 ----
            // 记录时间戳, 计算过零周期, 用于 30° 延迟和速度估算
            st->zc_detected = true;
            st->zc_confirm_ts = st->timer;
            st->zc_confirm_cnt = 0;

            // 更新时间戳: 上上次 ← 上次, 上次 ← 本次
            st->zc_ts_prev = st->zc_ts_cur;
            st->zc_ts_cur = st->timer;
            if (st->zc_ts_prev > 0.0f) {
              st->zc_period = st->zc_ts_cur - st->zc_ts_prev;
            }

            // 速度估算: 6 步/电周期, 每步之间有一次过零点
            // 过零周期 = 60° 电角度, 6 次过零 = 360° = 1 电周期
            // speed_est = 1 / (6 × zc_period)  (Hz, 电频率)
            if (st->zc_period > 1e-6f) {
              st->speed_est = 1.0f / (6.0f * st->zc_period);
            }
          }
        }
      } else if (crossed) {
        // 首次检测到有效符号变化 → 进入确认窗口
        st->zc_confirm_cnt = 1;
      }
      st->zc_diff_prev = diff;
    }

    // ---- 30° 延迟后换向 ----
    // 过零点位于两次换向的正中间 (30° 处),
    // 再延迟 30° 到达下一次换向位置 (60° 处)
    // delay_30deg = zc_period / 2
    if (st->zc_detected) {
      float delay_30deg;
      if (st->zc_period > 1e-6f) {
        delay_30deg = st->zc_period * 0.5f;
      } else {
        // zc_period 尚未建立 (仅检测到第一次过零), 用速度估算
        if (st->speed_est > 0.01f) {
          delay_30deg = 1.0f / (12.0f * st->speed_est);  // 30° = 1/12 电周期
        } else {
          delay_30deg = 0.01f;  // 兜底: 10ms
        }
      }

      if (st->timer - st->zc_confirm_ts >= delay_30deg) {
        // 换向到下一步
        st->commutation_step = (st->commutation_step + 1) % 6;
        st->last_comm_ts = st->timer;
        st->zc_detected = false;
        st->zc_confirm_cnt = 0;

        // 堵转计数器: 成功换向时重置
        // (注意: 如果 ZC 未检测到, 不会走到这里,
        //  则后面的堵转检测会累积并触发故障)
        st->stall_counter = 0;
      }
    }

    // ---- 堵转检测 ----
    // 如果当前步长时间未检测到过零点, 判定为堵转
    // 判定条件: 自上次换向以来经过的时间 > 预期的 2 倍换向间隔
    // 预期换向间隔 = 1 / (6 × speed_est), 但至少用 min_speed_hz 估算
    float expect_period;
    float ref_speed = (st->speed_est > cfg->min_speed_hz) ? st->speed_est : cfg->min_speed_hz;
    if (ref_speed > 0.01f) {
      expect_period = 1.0f / (6.0f * ref_speed);
    } else {
      expect_period = 0.1f;
    }
    // 如果超过 3 倍预期周期仍未检测到 ZC, 累积堵转计数
    if ((st->timer - st->last_comm_ts) > expect_period * 3.0f && !st->zc_detected) {
      st->stall_counter++;
      if (st->stall_counter >= cfg->stall_timeout_steps) {
        st->motor_state = BLDC_STATE_FAULT;
        st->fault_code = BLDC_FAULT_STALL;
        st->duty = 0.0f;
        return;
      }
    }

    // ---- 速度 PI 控制 ----
    // 误差 = 目标速度 - 估算速度
    float speed_err = st->speed_cmd - st->speed_est;

    // 比例项
    float p_term = cfg->kp_speed * speed_err;

    // 积分项 (带抗饱和)
    // 仅在占空比未饱和时累积积分, 防止积分器在限幅时无限增长
    st->pi_integral += cfg->ki_speed * speed_err * dt;

    // 限幅积分项到 [0, 1] 范围
    if (st->pi_integral > 1.0f) st->pi_integral = 1.0f;
    if (st->pi_integral < 0.0f) st->pi_integral = 0.0f;

    // PI 输出
    float pi_out = p_term + st->pi_integral;

    // 占空比限幅 [0, 1]
    if (pi_out > 1.0f) {
      pi_out = 1.0f;
      // 抗饱和: 输出已达上限, 不再累积正向积分
      if (speed_err > 0.0f) {
        st->pi_integral -= cfg->ki_speed * speed_err * dt;  // 回退本次积分增量
      }
    } else if (pi_out < 0.0f) {
      pi_out = 0.0f;
      // 抗饱和: 输出已达下限, 不再累积负向积分
      if (speed_err < 0.0f) {
        st->pi_integral -= cfg->ki_speed * speed_err * dt;  // 回退本次积分增量
      }
    }

    st->duty = pi_out;

    // ---- 堵转计数按周期累积 ----
    // 如果长时间未换向 (无 ZC), 累积计数
    // (已在上面 stall_counter 逻辑中处理)
    return;
  }
}

// ======== 访问器: 获取当前换向步 (0~5) ========
// 调用方根据换向步查真值表, 设置对应 PWM 通道的开关状态
static inline int bldc_instaspin_get_commutation_step(const BldcInstaSpinState *st) {
  return st->commutation_step;
}

// ======== 访问器: 获取当前 PWM 占空比 (0.0~1.0) ========
// 适用于 PWM+ 相的上管调制; PWM- 相的下管常通 (100% 或由死区控制)
static inline float bldc_instaspin_get_duty(const BldcInstaSpinState *st) {
  return st->duty;
}

// ======== 访问器: 获取估算电频率 (Hz) ========
// 机械转速 (RPM) = speed_est × 60 / pole_pairs
static inline float bldc_instaspin_get_speed(const BldcInstaSpinState *st) {
  return st->speed_est;
}

// ======== 访问器: 获取当前运行状态 ========
static inline BldcMotorState bldc_instaspin_get_motor_state(const BldcInstaSpinState *st) {
  return st->motor_state;
}

// ======== 访问器: 获取故障码 ========
static inline BldcFaultCode bldc_instaspin_get_fault_code(const BldcInstaSpinState *st) {
  return st->fault_code;
}

// ======== 访问器: 获取三相开关状态 ========
//
// 6 步换向真值表 (120° 导通, 方波驱动):
//   Step | A 相   | B 相   | C 相   | 悬空相
//   -----|--------|--------|--------|------
//    0   | PWM+   | PWM-   | Float  | C
//    1   | PWM+   | Float  | PWM-   | B
//    2   | Float  | PWM+   | PWM-   | A
//    3   | PWM-   | PWM+   | Float  | C
//    4   | PWM-   | Float  | PWM+   | B
//    5   | Float  | PWM-   | PWM+   | A
//
// 输出: phase_a/b/c = +1 (PWM+, 上管调制), -1 (PWM-, 下管导通), 0 (悬空, 上下管均关断)
//
// Device 层调用示例:
//   int pa, pb, pc;
//   bldc_instaspin_get_phase_state(&st, &pa, &pb, &pc);
//   if (pa == 1) { PWM_AH_SetDuty(duty); PWM_AL_SetDuty(0); }
//   if (pa == -1) { PWM_AH_SetDuty(0); PWM_AL_SetDuty(1); }  // 或互补
//   if (pa == 0) { PWM_AH_SetDuty(0); PWM_AL_SetDuty(0); }   // 高阻

static inline void bldc_instaspin_get_phase_state(const BldcInstaSpinState *st,
                                                   int *phase_a, int *phase_b, int *phase_c) {
  switch (st->commutation_step) {
    case 0: *phase_a =  1; *phase_b = -1; *phase_c =  0; break;  // A→B, 悬空C
    case 1: *phase_a =  1; *phase_b =  0; *phase_c = -1; break;  // A→C, 悬空B
    case 2: *phase_a =  0; *phase_b =  1; *phase_c = -1; break;  // B→C, 悬空A
    case 3: *phase_a = -1; *phase_b =  1; *phase_c =  0; break;  // B→A, 悬空C
    case 4: *phase_a = -1; *phase_b =  0; *phase_c =  1; break;  // C→A, 悬空B
    case 5: *phase_a =  0; *phase_b = -1; *phase_c =  1; break;  // C→B, 悬空A
    default: *phase_a =  0; *phase_b =  0; *phase_c =  0; break;  // 故障/未知
  }
}

// ======== 清除故障, 重新启动 ========
// 调用此函数后将回到 ALIGN 阶段, 重新执行对齐→开环→闭环流程
static inline void bldc_instaspin_clear_fault(BldcInstaSpinState *st) {
  st->motor_state = BLDC_STATE_ALIGN;
  st->fault_code = BLDC_FAULT_NONE;
  st->stall_counter = 0;
  st->stage_enter_ts = st->timer;
  st->commutation_step = 0;
  st->duty = 0.0f;
  st->pi_integral = 0.0f;
  st->zc_detected = false;
  st->zc_confirm_cnt = 0;
  st->zc_ts_prev = 0.0f;
  st->zc_ts_cur = 0.0f;
  st->zc_period = 0.0f;
  st->zc_diff_prev = 0.0f;
}

#endif  // COMP_BLDC_INSTASPIN_H
