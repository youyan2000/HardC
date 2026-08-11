#ifndef COMMS_H
#define COMMS_H

// 全局通信句柄 —— 应用层通过此文件访问所有通信实例
// 遵循分层架构: Application → Module → Devices → Components → BSP
//
// 用法:
//   #include "comms.h"
//   comm_send(c1, data, len);      // 通过 UART1 发送
//   comm_bgn(c2);                   // 启动超声波接收
//   comm_read(c4);                  // 读取 SPI 数据
//   can_send_msg((Can *)c6, 0x205, buf, 8);  // CAN 帧发送 (需向下转型)
//
// 句柄在 board_init.c 中由 board_comm_init() 绑定具体子类实例

#include "comp_comm.h"

// ======== 全局句柄声明 (由 board_init.c 定义) ========

// 串口类
extern CommBase *c1;  // UART1 — 蓝牙/上位机通信
extern CommBase *c2;  // UART3 — 超声波测距模块

// 显示类
extern CommBase *c3;  // OLED SSD1306 (软件 SPI)

// 总线类
extern CommBase *c4;  // SPI — 外部 SPI 设备
extern CommBase *c5;  // I2C — 外部 I2C 设备
extern CommBase *c6;  // CAN — CAN 总线

// 输入类
extern CommBase *k1;  // 按键1 (PA11)
extern CommBase *k2;  // 按键2 (PB0)
extern CommBase *k3;  // 按键3 (PB12)
extern CommBase *k4;  // 按键4 (PB13)

// 传感器类
extern CommBase *m1;  // MPU6050 六轴传感器 (软件 I2C)

#endif
