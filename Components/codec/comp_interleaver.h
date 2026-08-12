// 卷积交织器/解交织器 — 纯C环形缓冲实现, 配合 Reed-Solomon 纠错
//
// 来源: TI controlSUITE VCU/v2_10/common/interleaver.h + vcu2_deinterleaver.h
//
// 卷积交织原理 (Forney/Ramsey 型):
//   - branches 条分支 (行), 编号 0..branches-1
//   - 输入符号按循环顺序分配到各分支: 0,1,2,...,branches-1,0,1,...
//   - 分支 i 的延迟 = i*delay 符号 (交织器) 或 (branches-1-i)*delay 符号 (解交织器)
//   - 每条分支内部是一个 FIFO 环形缓冲区段
//   - 总缓冲容量 = delay*branches*(branches-1)/2 符号 (交织器和解交织器相同)
//   - 端到端通道延迟 = (branches-1)*delay 符号 (所有分支恒定, 互补抵消)
//   - 核心作用: 将信道突发错误分散到多个 RS 码字中, 化突发为随机
//
// 典型配置:
//   branches=12, delay=17 — DVB 标准卷积交织 (配合 RS(204,188) 纠错)
//   branches=16, delay=4  — 默认测试/演示配置
//
// 应用场景:
//   - 数字视频广播 (DVB-C/S/T): 信道编码链路中的外交织器
//   - 电力线通信 (PLC/G3-PLC): 电力线脉冲噪声抑制
//   - 无线充电通信 (Qi/NFC): 负载调制下的突发误码保护
//   - 深空通信: 配合级联码 (RS+卷积) 提升编码增益

#ifndef COMP_INTERLEAVER_H
#define COMP_INTERLEAVER_H

#include <stdint.h>

// ====================== 配置结构体 (纯数据, 可存 YAML) ======================

// 交织器/解交织器配置参数
// branches=分支数, delay=延迟增量, sym_size=每符号字节数
typedef struct {
  uint8_t branches;    // 分支数 (2-128), 典型值 12 (DVB) 或 16 (默认)
  uint8_t delay;       // 延迟增量 (1-32), 每分支比上一分支多 delay 个符号延迟
  uint8_t sym_size;    // 每符号字节数 (1-4), 符号宽度
} InterleaverCfg;

// ====================== 运行时状态结构体 ======================

// 交织器/解交织器运行时状态 — 共用结构体, 通过 is_deinterleaver 区分子类
// 所有动态内存由调用者外部分配 (零 malloc)
//
// 缓冲区布局 (线性):
//   [Branch 0段 (长度=0)] [Branch 1段 (长度=delay)] [Branch 2段 (长度=2*delay)] ...
//   每条分支内部是环形 FIFO: wptr 标记下一次读/写位置, 到达段尾回绕到段首
//
typedef struct {
  uint8_t  *buf;              // 线性环形缓冲区指针 (外部分配, 调用者提供)
  uint32_t  buf_bytes;        // 缓冲区总字节数 (= interleaver_get_buffer_size)
  uint32_t  branch_base[128]; // 每分支在 buf 中的基地址偏移 (字节, init 时预计算)
  uint16_t  wptr[128];        // 每分支当前写/读位置 (单位=符号, 范围 0..delay-1)
  uint16_t  branch;           // 当前活跃分支索引 [0, branches-1], 循环递增
  uint32_t  total_syms;       // 累计已处理的符号总数 (含数据和刷新的零符号)
  uint8_t   branches;         // 分支数 (来自配置)
  uint8_t   delay;            // 延迟增量 (来自配置)
  uint8_t   sym_size;         // 每符号字节数 (来自配置)
  uint8_t   is_deinterleaver; // 0=交织器模式, 1=解交织器模式
} Interleaver;

// ====================== 辅助: 分支延迟计算 ======================

// 计算指定分支的 FIFO 延迟 (符号数)
// 交织器:   分支 i 延迟 = i * delay          (递增延迟)
// 解交织器: 分支 i 延迟 = (branches-1-i) * delay    (递减延迟, 与交织器互补)
static inline uint16_t interleaver_branch_delay(const Interleaver *me, uint16_t b) {
  if (me->is_deinterleaver) {
    /* 解交织器: 分支0延迟最大=(branches-1)*delay, 分支branches-1延迟=0 */
    return (uint16_t)((int)me->branches - 1 - (int)b) * me->delay;
  } else {
    /* 交织器:   分支0延迟=0, 分支branches-1延迟最大=(branches-1)*delay */
    return (uint16_t)b * me->delay;
  }
}

// ====================== 默认配置 ======================

// 返回默认配置: branches=16 分支, delay=4 延迟增量, 1 字节/符号
// 总缓冲 = delay*branches*(branches-1)/2 = 4*16*15/2 = 480 符号 = 480 字节
static inline InterleaverCfg interleaver_cfg_default(void) {
  InterleaverCfg cfg;
  cfg.branches = 16;
  cfg.delay = 4;
  cfg.sym_size = 1;
  return cfg;
}

// ====================== 缓冲区大小计算 ======================

// 计算所需缓冲区容量 (符号数)
// 公式: delay * branches * (branches-1) / 2
// 推导: 分支 i 容量 = i*delay, 总和 = delay * sum(i, i=0..branches-1) = delay * branches*(branches-1)/2
static inline uint32_t interleaver_get_buffer_syms(const InterleaverCfg *cfg) {
  return (uint32_t)cfg->delay * cfg->branches * (cfg->branches - 1) / 2;
}

// 计算所需缓冲区容量 (字节数)
// 公式: 总符号数 * 每符号字节数
static inline uint32_t interleaver_get_buffer_size(const InterleaverCfg *cfg) {
  return interleaver_get_buffer_syms(cfg) * cfg->sym_size;
}

// ====================== 初始化 ======================

// 交织器初始化 — 配置分支延迟为递增模式 (分支 i 延迟 = i*delay)
// me:      状态结构体指针
// cfg:     配置参数
// buf:     外部分配的线性缓冲区 (大小 >= interleaver_get_buffer_size(cfg))
// buf_bytes: 缓冲区实际字节数
static inline void interleaver_init(Interleaver *me, const InterleaverCfg *cfg,
                                     uint8_t *buf, uint32_t buf_bytes) {
  me->buf = buf;
  me->buf_bytes = buf_bytes;
  me->branches = cfg->branches;
  me->delay = cfg->delay;
  me->sym_size = cfg->sym_size;
  me->is_deinterleaver = 0;
  me->branch = 0;
  me->total_syms = 0;

  /* 清零写指针和缓冲区 */
  for (int i = 0; i < me->branches; i++) {
    me->wptr[i] = 0;
    me->branch_base[i] = 0;
  }
  for (uint32_t i = 0; i < buf_bytes; i++) {
    me->buf[i] = 0;
  }

  /* 预计算每分支在 buf 中的基地址偏移 */
  uint32_t offset = 0;
  for (int i = 0; i < me->branches; i++) {
    me->branch_base[i] = offset;
    uint16_t delay = interleaver_branch_delay(me, (uint16_t)i);
    offset += (uint32_t)delay * me->sym_size;
  }
}

// 解交织器初始化 — 配置分支延迟为递减模式 (分支 i 延迟 = (branches-1-i)*delay)
// 参数同 interleaver_init, 区别在于延迟模式与交织器互补
static inline void deinterleaver_init(Interleaver *me, const InterleaverCfg *cfg,
                                       uint8_t *buf, uint32_t buf_bytes) {
  /* 先以交织器模式初始化, 再切换为解交织器并重新计算基地址 */
  interleaver_init(me, cfg, buf, buf_bytes);
  me->is_deinterleaver = 1;

  /* 重新计算基地址: 解交织器分支延迟模式与交织器镜像对称 */
  uint32_t offset = 0;
  for (int i = 0; i < me->branches; i++) {
    me->branch_base[i] = offset;
    uint16_t delay = interleaver_branch_delay(me, (uint16_t)i);
    offset += (uint32_t)delay * me->sym_size;
  }
}

// ====================== 重置 ======================

// 重置交织器状态 — 清空缓冲区, 复位所有指针
// 在开始新的数据块前调用, 或在 flush 之后准备下一轮
static inline void interleaver_reset(Interleaver *me) {
  for (int i = 0; i < me->branches; i++) {
    me->wptr[i] = 0;
  }
  for (uint32_t i = 0; i < me->buf_bytes; i++) {
    me->buf[i] = 0;
  }
  me->branch = 0;
  me->total_syms = 0;
}

// ====================== 写入 (交织/解交织核心) ======================

// 将一个符号写入交织器, 同时读出延迟后的符号
//
// 算法 (每条分支独立 FIFO):
//   1. 计算当前分支的延迟 delay (符号数)
//   2. 若 delay=0: 直通 — sym_out = sym_in (零延迟分支)
//   3. 若 delay>0: 读出 buf[pos] 中 delay 前写入的符号 → sym_out,
//      然后将新符号 sym_in 写入同一位置, 推进写指针 (环形回绕)
//   4. 切换到下一分支 (branch = (branch+1) % branches)
//
// sym_in:  输入符号缓冲区指针 (至少 sym_size 字节)
// sym_out: 输出符号缓冲区指针 (至少 sym_size 字节) — 接收延迟后的符号
static inline void interleaver_write(Interleaver *me, const uint8_t *sym_in, uint8_t *sym_out) {
  uint16_t b = me->branch;
  uint16_t delay = interleaver_branch_delay(me, b);

  if (delay == 0) {
    /* 零延迟分支: 直通, 不经过缓冲区 */
    for (int k = 0; k < me->sym_size; k++) {
      sym_out[k] = sym_in[k];
    }
  } else {
    /* 计算当前读写位置 (buf 中的绝对字节偏移) */
    uint32_t pos = me->branch_base[b] + (uint32_t)me->wptr[b] * me->sym_size;

    /* FIFO 先读后写: 读出延迟符号, 写入新符号 */
    for (int k = 0; k < me->sym_size; k++) {
      uint8_t tmp = me->buf[pos + k];
      me->buf[pos + k] = sym_in[k];
      sym_out[k] = tmp;
    }

    /* 推进写指针, 到达段尾则回绕到段首 (环形 FIFO) */
    me->wptr[b]++;
    if (me->wptr[b] >= delay) {
      me->wptr[b] = 0;
    }
  }

  /* 循环切换到下一分支 */
  me->branch++;
  if (me->branch >= me->branches) {
    me->branch = 0;
  }
  me->total_syms++;
}

// 解交织器写入 — 与交织器完全相同的核心逻辑
// 延迟模式由 init 时的 is_deinterleaver 标志决定, 因此直接复用 interleaver_write
static inline void deinterleaver_write(Interleaver *me, const uint8_t *sym_in, uint8_t *sym_out) {
  interleaver_write(me, sym_in, sym_out);
}

// ====================== 刷新 (块结束) ======================

// 块处理结束后, 刷新所有分支中残余的延迟符号
//
// 原理: 写入 branches*(branches-1)*delay 个零符号, 将管道中所有延迟的数据符号"推出"
//   输出的前 delay*branches*(branches-1)/2 个符号包含有效数据 (缓冲区存量),
//   后续为零符号 (刷新零已填满管道, 输出为零)
//
// sym_out:  输出缓冲区 (至少 max_syms * sym_size 字节)
// max_syms: 输出缓冲区可容纳的最大符号数
// 返回值:   实际输出的符号数 (<= max_syms)
//
// 使用示例:
//   for (int i = 0; i < n_data; i++) interleaver_write(&ilv, &in[i], &out[i]);
//   int n_flush = interleaver_flush(&ilv, &out[n_data], max_out - n_data);
//   interleaver_reset(&ilv);  // 准备下一块
static inline int interleaver_flush(Interleaver *me, uint8_t *sym_out, int max_syms) {
  /* 需要推入的零符号总数: 每条分支访问 (branches-1)*delay 次, 共 branches 条分支 */
  uint32_t flush_needed = (uint32_t)me->branches * (me->branches - 1) * me->delay;
  int count = 0;
  uint8_t zero[4] = {0, 0, 0, 0};

  while (count < max_syms && (uint32_t)count < flush_needed) {
    interleaver_write(me, zero, sym_out);
    sym_out += me->sym_size;
    count++;
  }
  return count;
}

// 解交织器刷新 — 同 interleaver_flush, 包装函数
static inline int deinterleaver_flush(Interleaver *me, uint8_t *sym_out, int max_syms) {
  return interleaver_flush(me, sym_out, max_syms);
}

#endif  // COMP_INTERLEAVER_H
