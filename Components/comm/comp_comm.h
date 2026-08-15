#ifndef COMP_COMM_H
#define COMP_COMM_H

// 通信契约身份基类 — CommBase 只承载契约身份 + 生命周期 + 诊断
//
// 定位: 不绑定任何传输/设备/协议, 数据面在子类 (Uart/Spi/I2c/Can/Gpio, Devices/comm)
// 重构来源: 旧版 CommBase 把传输/设备/协议三层压平成"字节流虚表" (send/bgn/read)
//   + 实例名枚举, 9 个子类只有 2 个真用 → 瘦身为契约身份
//
// 数据视图: CommData/CommConstData — LibXR RawData/ConstRawData 平替
//
// 诊断虚表 CommOps: self_check/reset 均 [可选], 可空
//   comm_self_check: ops 或 self_check 为空 → 返回 0 (通过)
//   comm_reset:      ops 或 reset 为空 → 忽略

#include "comp_io.h"
#include "comp_error_code.h"
#include <stdint.h>

typedef struct CommBase CommBase;

// 数据视图 — LibXR RawData 平替 (可写)
typedef struct {
  uint8_t *ptr;  // 数据指针
  uint16_t len;  // 数据长度
} CommData;

// 常量数据视图 — LibXR ConstRawData 平替 (只读)
typedef struct {
  const uint8_t *ptr;  // 数据指针
  uint16_t len;        // 数据长度
} CommConstData;

// 诊断虚函数表 ([可选], 可空)
typedef struct {
  int (*self_check)(CommBase *me);  // [可选] 自检, 0=通过
  void (*reset)(CommBase *me);      // [可选] 复位
} CommOps;

// 基类 — 契约身份
struct CommBase {
  const char *name;    // 实例名 (调试/诊断用)
  uint8_t inited;      // 初始化标志 (0=未初始化, 1=已初始化)
  const CommOps *ops;  // 诊断虚表 (可空)
};

// 基类构造 / 析构
void comm_base_init(CommBase *me, const char *name);
void comm_base_deinit(CommBase *me);

// 诊断分发 —— 通过 ops 调用子类实现 (均为空安全)
int comm_self_check(CommBase *me);  // 自检: 未绑定或未实现 → 返回 0
void comm_reset(CommBase *me);      // 复位: 未绑定或未实现 → 忽略

#endif
