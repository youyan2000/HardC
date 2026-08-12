// 控制器自动调参 — 触发式阶跃响应捕获 + 性能准则 (DCL 库 TCM)
//
// 来源: TI C2000Ware Digital Power SDK c2000ware/libraries/control/DCL/c28/include/DCL_TCM.h
// 翻译为 C-OOP 纯C float 版本 (FDLOG 环形日志 → 自包含环形缓冲)
//
// TCM (Tuning Criteria Module) 工作流:
//   1. 初始化: 捕获窗 size 样本, 其中 lead 个为触发前 (预触发) 样本
//   2. armed: 连续写入预触发环形缓冲, 检测误差信号越限 (|e|>trigMax 或 <trigMin)
//   3. 触发: 回填 lead 个预触发样本 → 继续捕获后触发样本 → 窗口满 → complete
//   4. 对捕获的误差响应计算性能准则:
//      IAE  = Σ|e[k]|                  (积分绝对误差)
//      ISE  = Σe[k]²                   (积分平方误差)
//      ITAE = Σ|e[k]|·t[k]             (时间加权绝对误差)
//   5. 在候选增益中重放捕获响应, 取准则最小者为最优 (自动调参)
//
// 用法 (自动调参循环):
//   for (每个候选增益) {
//     tcm_arm(&tcm); while (tcm.state != TCM_COMPLETE) tcm_run(&tcm, err);
//     score = tcm_itae(tcm.buf, tcm.n, dt);
//     保留 score 最小者;
//   }

#ifndef COMP_TCM_H
#define COMP_TCM_H

#include <stdint.h>
#include <math.h>

// 预触发环形缓冲大小上限 (用户可调, 须 ≥ lead)
#define TCM_PRE_MAX  256u

// ======================= 状态 =======================

typedef enum {
  TCM_IDLE = 0,     // 空闲 (未 armed)
  TCM_ARMED,        // 已 armed: 预触发环形缓冲持续写入, 等待触发
  TCM_CAPTURE,      // 已触发: 捕获后触发样本
  TCM_COMPLETE      // 捕获窗满, 数据可用 (buf[0..n-1])
} TcmState;

// ======================= 捕获模块 =======================

typedef struct {
  // 参数
  float *buf;             // 输出: 用户提供的捕获缓冲 (大小 ≥ size)
  uint16_t size;          // 参数: 捕获窗总样本数 (预触发 lead + 后触发)
  uint16_t lead;          // 参数: 预触发样本数
  float trig_max;         // 参数: 上触发阈值 (误差越限触发)
  float trig_min;         // 参数: 下触发阈值

  // 预触发环形缓冲 (自包含)
  float pre[TCM_PRE_MAX];
  uint16_t pre_wptr;

  // 状态
  TcmState state;
  uint16_t n;             // 输出: 已捕获样本数 (= size 时 complete)
  uint16_t cap_idx;       // 捕获写指针
} TcmCapture;

// 初始化
//   buf      — 捕获缓冲 (调用者提供, 长度 ≥ size)
//   lead     — 预触发样本数
//   size     — 捕获窗总长度 (lead ≤ size ≤ TCM_PRE_MAX)
//   trig_min/trig_max — 触发阈值 (误差越限触发捕获)
static inline void tcm_init(TcmCapture *me, float *buf, uint16_t lead,
                            uint16_t size, float trig_min, float trig_max) {
  me->buf = buf;
  me->lead = (lead == 0u) ? 1u : lead;   // 防止取模除零
  me->size = size;
  me->trig_max = trig_max;
  me->trig_min = trig_min;

  // 预触发环形缓冲清零 — 避免触发早于 lead 个样本时回填未初始化内存
  for (uint16_t i = 0u; i < TCM_PRE_MAX; i++) {
    me->pre[i] = 0.0f;
  }
  me->pre_wptr = 0u;
  me->state = TCM_IDLE;
  me->n = 0u;
  me->cap_idx = 0u;
}

// armed ←→ 切换捕获使能 (从 idle 进入 armed)
static inline void tcm_arm(TcmCapture *me) {
  if (me->state == TCM_IDLE) {
    me->state = TCM_ARMED;
  }
}

// 取消捕获, 回到空闲
static inline void tcm_disarm(TcmCapture *me) {
  me->state = TCM_IDLE;
  me->n = 0u;
  me->cap_idx = 0u;
}

// 单步运行 — 每个采样周期传入误差信号 e
// 触发顺序与 DCL_runTCM 一致: 先判定触发, 触发样本作为首个后触发样本,
// 预触发环形缓冲回填至捕获窗头部 (lead 个样本为触发前的稳态响应)
static inline void tcm_run(TcmCapture *me, float e) {
  switch (me->state) {
    case TCM_ARMED:
      // 越限触发?
      if (e > me->trig_max || e < me->trig_min) {
        // 回填预触发窗口: 环形缓冲按 最老→最新 顺序 → buf[0..lead-1]
        me->cap_idx = 0u;
        for (uint16_t i = 0u; i < me->lead; i++) {
          me->buf[me->cap_idx++] = me->pre[(me->pre_wptr + i) % me->lead];
        }
        me->buf[me->cap_idx++] = e;   // 触发样本 → 后触发区首位
        me->state = TCM_CAPTURE;
      } else {
        // 未触发: 写入预触发环形缓冲
        me->pre[me->pre_wptr] = e;
        me->pre_wptr = (me->pre_wptr + 1u) % me->lead;
      }
      break;

    case TCM_CAPTURE:
      // 继续捕获后触发样本
      me->buf[me->cap_idx++] = e;
      if (me->cap_idx >= me->size) {
        me->state = TCM_COMPLETE;
        me->n = me->size;
      }
      break;

    default:
      break;  // IDLE / COMPLETE: 不做任何事
  }
}

// ======================= 性能准则 (对捕获的误差响应求值) =======================

// IAE — 积分绝对误差: Σ|e[k]|
static inline float tcm_iae(const float *err, uint16_t n) {
  float s = 0.0f;
  for (uint16_t k = 0u; k < n; k++) {
    s += fabsf(err[k]);
  }
  return s;
}

// ISE — 积分平方误差: Σe[k]²
static inline float tcm_ise(const float *err, uint16_t n) {
  float s = 0.0f;
  for (uint16_t k = 0u; k < n; k++) {
    float e = err[k];
    s += e * e;
  }
  return s;
}

// ITAE — 时间加权绝对误差: Σ|e[k]|·t[k], t[k]=k·dt (加权尾部长衰减)
static inline float tcm_itae(const float *err, uint16_t n, float dt) {
  float s = 0.0f;
  float t = 0.0f;
  for (uint16_t k = 0u; k < n; k++) {
    s += fabsf(err[k]) * t;
    t += dt;
  }
  return s;
}

#endif  // COMP_TCM_H
