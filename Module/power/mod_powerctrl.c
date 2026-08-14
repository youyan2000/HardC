// 功率控制状态机实现 — PWM-OOP Module 层
// Module 层状态机模式
//
// 状态转换图:
//   INIT ──(自检OK)──→ IDLE ──(start)──→ RUN ──(故障检测)──→ FAULT_HOLD
//   FAULT_HOLD ──(去抖动确认)──→ FAULT ──(延迟)──→ RECOVER ──(软启完成)──→ IDLE
//   ANY ──(emergency)──→ FAULT
//
// 使用方式:
//   1. 用户创建子类结构体, ModPwr 为第一个成员
//   2. 构造时调用 mod_pwr_init(), 再绑定自己的 Device/Component 指针
//   3. 在 tick 中实现具体控制逻辑 (采样→滤波→PID→PWM)
//   4. 故障检测条件由用户根据硬件定义

#include "mod_powerctrl.h"
#include <string.h>

// ======== 构造 ========

void mod_pwr_init(ModPwr *me, const ModPwr_Param *param) {
  memset(me, 0, sizeof(*me));
  me->st      = PCTRL_INIT;
  me->st_prev = PCTRL_INIT;
  if (param) {
    me->param = *param;
  }
  me->fault_debounce_cnt = 0;
}

// ======== 状态机驱动 (每 tick 调用一次) ========

/*
  每 tick 流程:

  PCTRL_INIT:
    1. 硬件自检: PWM 通道是否就绪 / ADC 是否有数据 / 通信是否正常
    2. 自检通过 → PCTRL_IDLE
    3. 自检失败 → 重试或上报错误

  PCTRL_IDLE:
    1. 心跳更新 (通过 status_ 指针)
    2. LED 状态指示
    3. 通信响应 (通过 conn_ 指针)
    4. 等待 start() 命令

  PCTRL_RUN:
    1. 采样 (通过 sampler_ 获取电压/电流)
    2. 滤波 (通过 filt_v_ 低通滤波)
    3. 模式切换检测:
      - 进入条件 (窄窗口): 如 v < v_enter_bb → BuckBoost 模式
      - 退出条件 (宽窗口): 如 v > v_exit_bb → Buck/Boost 模式
      - 滞回防止模式振荡
    4. 模式切换单周期同步:
      if (st != st_prev) {
        三相统一平均占空比跑一个周期
        st_prev = st;
        return;  // 本 tick 不再更新
      }
    5. PID 计算 (通过 pid_v_)
    6. 输出限幅 (hard clamp: duty_min ≤ output ≤ duty_max)
    7. PWM 更新 (通过 pwm_set_duty)

    故障检测:
    - 短路: 电流 > 硬件阈值
    - 过流: 电流 > 软件阈值持续
    - 过压: 电压 > 安全阈值
    - 通信超时: conn_ 无心跳
    → fault_debounce_cnt++
    → if (fault_debounce_cnt > param.fault_debounce_s / dt) → PCTRL_FAULT_HOLD

  PCTRL_FAULT_HOLD:
    去抖动确认:
    - 再连续采样 N 次
    - 依然触发 → PCTRL_FAULT (封波 + 记录故障码)
    - 不再触发 → 恢复 PCTRL_RUN (误触发, 清除 debounce 计数)

  PCTRL_FAULT:
    1. 封波: pwm_emergency_stop() 或 duty=0
    2. 记录故障码 (通过 status_ 指针, ERROR_SET 宏)
    3. CAN/串口上报故障 (通过 conn_ 指针)
    4. 等待 recover_delay_s 后 → PCTRL_RECOVER

  PCTRL_RECOVER:
    1. 清除故障码 (通过 status_ 指针)
    2. 逐步恢复 (软启动: duty 从 0 逐步升到目标值, 步长 soft_start_step)
    3. 软启完成 → PCTRL_IDLE
*/
void mod_pwr_tick(ModPwr *me, float dt) {
  me->tick_cnt++;

  switch (me->st) {

  case PCTRL_INIT:
    // TODO: 用户在此插入硬件自检逻辑
    // if (pwm_ready && adc_ready && comm_ready) {
    //   me->st = PCTRL_IDLE;
    // }
    me->st = PCTRL_IDLE;
    break;

  case PCTRL_IDLE:
    // TODO: 用户在此插入空闲逻辑
    // - 心跳更新
    // - LED 状态指示
    // - 通信响应
    break;

  case PCTRL_RUN: {
    // === 主控制循环 (用户实现) ===

    // 1. 采样
    // float v = sampler_->read_voltage();
    // float i = sampler_->read_current();

    // 2. 滤波
    // float v_filt = lpf_tick(&filt_v_, v, dt);

    // 3. 模式切换检测 (滞回)
    // if (v_filt < param.v_enter_bb) {
    //   device_mode = BuckBoost;
    // } else if (v_filt > param.v_exit_bb) {
    //   device_mode = Buck;
    // }
    // 注: 进入窗口窄 (v_enter_bb), 退出窗口宽 (v_exit_bb), 防振荡

    // 4. 模式切换单周期同步
    // if (me->st != me->st_prev) {
    //   // 三相统一平均占空比 (如三相同时切模式时用上一周期平均)
    //   pwm_set_duty(buckboost_, 0, last_duty);
    //   me->st_prev = me->st;
    //   return;
    // }

    // 5. PID 计算
    // pid_set(&pid_v_, vref, v_filt);
    // float duty = pid_calc(&pid_v_);

    // 6. 输出限幅 (在 pwm_set_duty 内自动 clamp 到 [duty_min, duty_max])
    // me->last_duty = duty;
    // pwm_set_duty(buckboost_, 0, duty);

    // === 故障检测 (用户定义具体阈值) ===
    // if (overcurrent || overvoltage || comm_timeout) {
    //   me->fault_debounce_cnt++;
    //   if (me->fault_debounce_cnt > me->param.fault_debounce_s / dt) {
    //     me->st = PCTRL_FAULT_HOLD;
    //     me->fault_debounce_cnt = 0;
    //   }
    // } else {
    //   me->fault_debounce_cnt = 0;
    // }
    break;
  }

  case PCTRL_FAULT_HOLD:
    // TODO: 去抖动确认
    // 再连续采样 N 次
    // if (still_fault) {
    //   me->st = PCTRL_FAULT;
    // } else {
    //   me->st = PCTRL_RUN;
    //   me->fault_debounce_cnt = 0;
    // }
    break;

  case PCTRL_FAULT:
    // TODO: 故障处理
    // 1. 封波
    // pwm_emergency_stop(...) 或 duty = 0
    // 2. 记录故障码
    // ERROR_SET(status_->errors, ERR_OVERCURRENT);
    // 3. 上报故障 (CAN/串口)
    // conn_->send_fault(status_->errors);

    // 等待恢复延迟
    // static uint32_t fault_tick = 0;
    // fault_tick++;
    // if (fault_tick * dt > me->param.recover_delay_s) {
    //   me->st = PCTRL_RECOVER;
    //   fault_tick = 0;
    // }
    break;

  case PCTRL_RECOVER:
    // TODO: 故障恢复
    // 1. 清除故障码
    // ERROR_CLEAR(status_->errors, ERR_OVERCURRENT);
    // 2. 软启动: duty 从 0 阶梯式升到目标值
    // static float ramp_duty = 0;
    // ramp_duty += param.soft_start_step;
    // if (ramp_duty >= target_duty) {
    //   ramp_duty = 0;
    //   me->st = PCTRL_IDLE;
    // }
    break;

  default:
    break;
  }
}

// ======== 控制接口 ========

void mod_pwr_start(ModPwr *me) {
  if (me->st == PCTRL_IDLE || me->st == PCTRL_RECOVER) {
    me->st = PCTRL_RUN;
  }
}

void mod_pwr_stop(ModPwr *me) {
  // 正常停机流程:
  // 1. 斜坡降输出 (如果子类实现了斜坡逻辑)
  // 2. 封波
  // 3. 回到 IDLE
  me->st = PCTRL_IDLE;
  me->last_duty = 0.0f;
}

void mod_pwr_emergency_stop(ModPwr *me) {
  // 紧急封波: 立即清零 PWM, 不经过斜坡
  // 用户在 tick 的 FAULT 状态中调用 pwm_emergency_stop()
  me->st = PCTRL_FAULT;
  me->last_duty = 0.0f;
}

// ======== 查询接口 ========

bool mod_pwr_is_fault(const ModPwr *me) {
  return me->st == PCTRL_FAULT || me->st == PCTRL_FAULT_HOLD;
}

PwrSt mod_pwr_get_state(const ModPwr *me) {
  return me->st;
}
