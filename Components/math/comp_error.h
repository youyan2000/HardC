#ifndef COMP_ERROR_H
#define COMP_ERROR_H

// 嵌入式 OOP 统一错误码系统 —— bitmask 模式
//
// 使用方式:
//   uint8_t err = 0;
//   ERROR_SET(err, ERROR_UNDER_VOLTAGE);
//   if (ERROR_IS_SET(err, ERROR_SHORT_CIRCUIT)) { ... }
//   ERROR_CLEAR(err, ERROR_UNDER_VOLTAGE);
//
// 设计原则:
//   - 每个模块定义自己的错误位 (不冲突)
//   - bitmask 支持同时多个错误
//   - 去抖动计数在 Module 层实现 (不在 Components 层)

#include <stdint.h>

// ======== 通用错误位定义 (0~7, 供所有模块使用) ========

// WARNING 级别 (不妨碍运行, 但需要上报)
#define ERROR_UNDER_VOLTAGE     (1 << 0)  // 欠压
#define ERROR_OVER_VOLTAGE      (1 << 1)  // 过压
#define ERROR_OVER_TEMP         (1 << 2)  // 过温

// FAULT 级别 (需要停机/保护)
#define ERROR_SHORT_CIRCUIT     (1 << 3)  // 短路
#define ERROR_OVER_CURRENT      (1 << 4)  // 过流
#define ERROR_NO_POWER_INPUT    (1 << 5)  // 无输入电源
#define ERROR_PHASE_UNBALANCE   (1 << 6)  // 多相不均流
#define ERROR_COMM_TIMEOUT      (1 << 7)  // 通信超时

// ======== 位操作宏 ========

// 置位
#define ERROR_SET(code, bit)    ((code) |= (bit))

// 清除
#define ERROR_CLEAR(code, bit)  ((code) &= (uint8_t)(~(bit)))

// 检查是否置位: 返回非零 = 置位
#define ERROR_IS_SET(code, bit) (((code) & (bit)) != 0U)

// 检查是否有任何错误
#define ERROR_ANY(code)         ((code) != 0U)

// 全部清零
#define ERROR_CLEAR_ALL(code)   ((code) = 0U)

#endif
