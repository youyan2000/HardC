// 错误码枚举 — LibXR ErrorCode 的纯 C 平替 (I/O 调用返回值 / API 结果)
//
// 来源: LibXR `enum class ErrorCode : int8_t` (libxr_def.hpp), 数值完全对齐
//
// 约定:
//   ERR_OK = 0 开头, 非零为错误
//   ERR_PENDING = 1 表示"等待中" (非错误, 操作未完成)
//   负数表示各类错误
//
// 与 comp_error.h (math) 的分工:
//   ErrorCode    — I/O 调用返回值 / API 结果 (per-call 临时值, 函数返回)
//   comp_error.h — 运行时保护分级位 (bitmask, 持久状态: 欠压/过流/短路/超时)
//   前缀避让: ErrorCode 用 ERR_, comp_error.h 用 ERROR_, 互不冲突
//
// 用法:
//   ErrorCode ec = drv_read(...);
//   if (ec != ERR_OK) { ... }

#ifndef COMP_ERROR_CODE_H
#define COMP_ERROR_CODE_H

typedef enum {
  ERR_PENDING = 1,        // 等待中 (操作未完成, 非错误)
  ERR_OK = 0,             // 操作成功
  ERR_FAILED = -1,        // 操作失败
  ERR_INIT = -2,          // 初始化错误
  ERR_ARG = -3,           // 参数错误
  ERR_STATE = -4,         // 状态错误
  ERR_SIZE = -5,          // 尺寸错误
  ERR_CHECK = -6,         // 校验错误
  ERR_NOT_SUPPORT = -7,   // 不支持
  ERR_NOT_FOUND = -8,     // 未找到
  ERR_NO_RESPONSE = -9,   // 无响应
  ERR_NO_MEM = -10,       // 内存不足
  ERR_NO_BUFF = -11,      // 缓冲区不足
  ERR_TIMEOUT = -12,      // 超时
  ERR_EMPTY = -13,        // 为空
  ERR_FULL = -14,         // 已满
  ERR_BUSY = -15,         // 忙碌
  ERR_PTR_NULL = -16,     // 空指针
  ERR_OUT_OF_RANGE = -17  // 超出范围
} ErrorCode;

#endif  // COMP_ERROR_CODE_H
