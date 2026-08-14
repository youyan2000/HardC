// 数据记录器 — ISR 实时数据捕获环形缓冲区 (纯C, 无硬件依赖)
//
// 来源: TI controlSUITE solar/v1.2/float (DLOG_1CH_F.h, DLOG_4CH_F.h)
// 翻译为 HardC 纯C float static inline 版本
//
// 两种变体:
//   Dlog1ch  — 单通道数据记录器 (单个 float 信号)
//   Dlog4ch  — 四通道数据记录器 (4 个信号同时捕获, 交替存储)
//
// 核心机制:
//   - 环形缓冲区: wptr 写指针回绕, full 标志表示已完成一整圈
//   - 预分频器: 每 N 次 ISR 调用捕获一次 (prescale=N)
//   - 触发: trigger() 启动 → 捕获直到缓冲区满 → 自动停止 (oneshot 模式)
//   - Dlog4ch 额外支持自动触发: 某通道电平越限时自动启动
//
// 应用场景:
//   - 电源环路调试: 捕获输出电压/电流波形 (ISR 内记录, 后台打印)
//   - 故障录波: 触发条件满足时记录故障前后波形
//   - PID 调参辅助: 捕获设定值/反馈/输出三路信号
//   - 扫频分析: 配合 SgenSweep 记录频率响应

#ifndef COMP_DLOG_H
#define COMP_DLOG_H

#include <stdbool.h>
#include <stddef.h>

// ======================= Dlog1ch (单通道数据记录器) =======================

// 环形缓冲区, 预分频, 触发后捕获一满缓冲即停止
//
// 使用示例 (ISR):
//   Dlog1ch log;
//   float buf[1024];
//   dlog1ch_init(&log, buf, 1024, 1);   // 每次 ISR 都记录
//   dlog1ch_trigger(&log);              // 启动捕获
//
//   // ISR 内:
//   dlog1ch_capture(&log, adc_voltage);  // 满 1024 点后自动停止
//
//   // 后台打印:
//   int n = dlog1ch_count(&log);
//   for (int i = 0; i < n; i++) {
//     printf("%.4f\n", dlog1ch_read(&log, i));
//   }

typedef struct {
  float *buf;           // 用户提供的缓冲区 (外部分配, 零 malloc)
  int    size;          // 缓冲区容量 (采样点数)
  int    wptr;          // 写指针: 下一个写入位置 [0, size-1]
  int    count;         // 累计写入次数 (仅 full=false 时有意义)
  int    prescale;      // 预分频值: 每 N 次 capture 调用写入一次 (<=1 = 每次)
  int    prescale_cnt;  // 预分频计数器
  bool   full;          // true = 缓冲区已满 (已回绕至少一次)
  bool   triggered;     // true = 正在捕获中
} Dlog1ch;

// 初始化: 绑定缓冲区, 设置容量和预分频, 所有状态清零
static inline void dlog1ch_init(Dlog1ch *me, float *buf, int size, int prescale) {
  me->buf = buf;
  me->size = size;
  me->wptr = 0;
  me->count = 0;
  me->prescale = prescale;
  me->prescale_cnt = 0;
  me->full = false;
  me->triggered = false;
}

// 强制启动捕获 (重置内部状态, 开始新一轮记录)
static inline void dlog1ch_trigger(Dlog1ch *me) {
  me->wptr = 0;
  me->count = 0;
  me->prescale_cnt = 0;
  me->full = false;
  me->triggered = true;
}

// 停止捕获 (保留已记录的数据)
static inline void dlog1ch_stop(Dlog1ch *me) {
  me->triggered = false;
}

// ISR 内调用: 若已触发且预分频到期, 写入 val 到环形缓冲区
//   缓冲区满后自动停止 (oneshot 模式)
static inline void dlog1ch_capture(Dlog1ch *me, float val) {
  if (!me->triggered) {
    return;
  }

  // 预分频: 每 prescale 次调用才写入一次
  if (me->prescale > 1) {
    me->prescale_cnt++;
    if (me->prescale_cnt < me->prescale) {
      return;
    }
    me->prescale_cnt = 0;
  }

  me->buf[me->wptr] = val;
  me->wptr++;
  me->count++;

  // 环形回绕
  if (me->wptr >= me->size) {
    me->wptr = 0;
    me->full = true;
    me->triggered = false;  // oneshot: 满即停
  }
}

// 有效采样点数: full ? size : count
static inline int dlog1ch_count(const Dlog1ch *me) {
  return me->full ? me->size : me->count;
}

// 逻辑读取 (时间顺序, idx=0 为最旧):
//   - 已回绕 (full=true):  idx=0 → buf[wptr] (最旧=下一个即将覆盖的位置)
//   - 未回绕 (full=false): idx=0 → buf[0]
static inline float dlog1ch_read(const Dlog1ch *me, int idx) {
  if (me->full) {
    return me->buf[(me->wptr + idx) % me->size];
  } else {
    return me->buf[idx];
  }
}

// ======================= Dlog4ch (四通道数据记录器) =======================

// 四通道同时捕获, 交替存储: buf = [ch0,ch1,ch2,ch3, ch0,ch1,ch2,ch3, ...]
// 缓冲区大小 = size × 4 (size 为每通道采样点数)
//
// 支持自动触发: 当 trigger_ch 指定的通道穿越 trigger_level 时自动启动
//
// 使用示例 (ISR):
//   Dlog4ch log;
//   float buf[1024 * 4];
//   dlog4ch_init(&log, buf, 1024, 1);     // 每通道 1024 点
//   dlog4ch_set_auto_trig(&log, 0, 1.5f, 0); // ch0 超过 1.5V 时自动触发 (上升沿)
//   // 或手动触发:
//   dlog4ch_trigger(&log);
//
//   // ISR 内:
//   dlog4ch_capture(&log, vout, iout, vin, temp);
//
//   // 后台读取 ch0:
//   int n = dlog4ch_count(&log);
//   for (int i = 0; i < n; i++) {
//     printf("%.4f\n", dlog4ch_read(&log, 0, i));
//   }

typedef struct {
  float *buf;           // 用户提供的交替缓冲区 (buf[4×size], 外部分配)
  int    size;          // 每通道采样点数 (buf 总容量 = size × 4)
  int    wptr;          // 写指针: 下一个写入位置 (单位=采样组, [0, size-1])
  int    count;         // 累计写入次数 (采样组数)
  int    prescale;      // 预分频值 (<=1 = 每次)
  int    prescale_cnt;  // 预分频计数器
  bool   full;          // true = 缓冲区已满
  bool   triggered;     // true = 正在捕获中

  // 自动触发 (可选)
  bool  auto_trig_en;   // true = 启用自动触发
  int   trigger_ch;     // 触发源通道号 (0~3)
  float trigger_level;  // 触发电平
  int   trigger_dir;    // 0=上升沿 (穿越上方), 1=下降沿 (穿越下方)
  float prev_trig_val;  // 触发通道上一帧的值 (用于沿检测)
} Dlog4ch;

// 初始化: 绑定交替缓冲区, 设置每通道容量和预分频, 所有状态清零
static inline void dlog4ch_init(Dlog4ch *me, float *buf, int size, int prescale) {
  me->buf = buf;
  me->size = size;
  me->wptr = 0;
  me->count = 0;
  me->prescale = prescale;
  me->prescale_cnt = 0;
  me->full = false;
  me->triggered = false;
  me->auto_trig_en = false;
  me->trigger_ch = 0;
  me->trigger_level = 0.0f;
  me->trigger_dir = 0;
  me->prev_trig_val = 0.0f;
}

// 配置自动触发: ch=触发通道, level=阈值, dir=0上升沿/1下降沿
static inline void dlog4ch_set_auto_trig(Dlog4ch *me, int ch, float level, int dir) {
  me->auto_trig_en = true;
  me->trigger_ch = ch;
  me->trigger_level = level;
  me->trigger_dir = dir;
  me->prev_trig_val = 0.0f;
}

// 禁用自动触发 (恢复手动触发模式)
static inline void dlog4ch_disable_auto_trig(Dlog4ch *me) {
  me->auto_trig_en = false;
}

// 强制启动捕获 (重置内部状态, 开始新一轮记录)
static inline void dlog4ch_trigger(Dlog4ch *me) {
  me->wptr = 0;
  me->count = 0;
  me->prescale_cnt = 0;
  me->full = false;
  me->triggered = true;
}

// 停止捕获 (保留已记录的数据)
static inline void dlog4ch_stop(Dlog4ch *me) {
  me->triggered = false;
}

// 辅助: 根据 trigger_ch 从 4 个通道值中提取触发源
static inline float dlog4ch_trig_val(const Dlog4ch *me,
                                     float ch0, float ch1, float ch2, float ch3) {
  switch (me->trigger_ch) {
    case 0:  return ch0;
    case 1:  return ch1;
    case 2:  return ch2;
    default: return ch3;
  }
}

// ISR 内调用: 自动触发检测 → 预分频 → 写入 4 通道到交替缓冲区
//   缓冲区满后自动停止 (oneshot 模式)
static inline void dlog4ch_capture(Dlog4ch *me, float ch0, float ch1, float ch2, float ch3) {
  // 自动触发检测 (仅在未触发且启用自动触发时)
  if (me->auto_trig_en && !me->triggered) {
    float cur = dlog4ch_trig_val(me, ch0, ch1, ch2, ch3);
    bool edge = false;
    if (me->trigger_dir == 0) {
      // 上升沿: 之前低于阈值, 当前 >= 阈值
      edge = (me->prev_trig_val < me->trigger_level && cur >= me->trigger_level);
    } else {
      // 下降沿: 之前高于阈值, 当前 <= 阈值
      edge = (me->prev_trig_val > me->trigger_level && cur <= me->trigger_level);
    }
    me->prev_trig_val = cur;
    if (edge) {
      dlog4ch_trigger(me);  // 沿触发: 启动捕获
    }
  }

  if (!me->triggered) {
    return;
  }

  // 预分频
  if (me->prescale > 1) {
    me->prescale_cnt++;
    if (me->prescale_cnt < me->prescale) {
      return;
    }
    me->prescale_cnt = 0;
  }

  // 交替写入: buf[wptr*4 + ch]
  int base = me->wptr * 4;
  me->buf[base + 0] = ch0;
  me->buf[base + 1] = ch1;
  me->buf[base + 2] = ch2;
  me->buf[base + 3] = ch3;

  me->wptr++;
  me->count++;

  // 环形回绕
  if (me->wptr >= me->size) {
    me->wptr = 0;
    me->full = true;
    me->triggered = false;  // oneshot: 满即停
  }
}

// 有效采样点数 (每通道): full ? size : count
static inline int dlog4ch_count(const Dlog4ch *me) {
  return me->full ? me->size : me->count;
}

// 逻辑读取指定通道的时间序列 (ch=0~3, idx=0 为最旧):
//   - 已回绕 (full=true):  idx=0 → buf[(wptr+idx)%size * 4 + ch]
//   - 未回绕 (full=false): idx=0 → buf[idx * 4 + ch]
static inline float dlog4ch_read(const Dlog4ch *me, int ch, int idx) {
  int logical;
  if (me->full) {
    logical = (me->wptr + idx) % me->size;
  } else {
    logical = idx;
  }
  return me->buf[logical * 4 + ch];
}

#endif  // COMP_DLOG_H
