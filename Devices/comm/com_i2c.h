#ifndef COM_I2C_H
#define COM_I2C_H

// I2C 通信驱动 —— CommBase 的子类
// 主模式 I2C, 设备地址在 init 时绑定
// send: HAL_I2C_Master_Transmit 阻塞写入
// bgn:  HAL_I2C_Master_Receive_IT 启动单字节中断接收
// read: 返回最近收到的字节
//
// 扩展 API:
//   iic_read_reg() — 写寄存器地址 → 读 N 字节（传感器寄存器访问模式）

#include "comp_comm.h"
#include "stm32f1xx_hal.h"

// 子类结构体 —— base 必须是第一个成员（保证 &iic.base == &iic）
typedef struct {
  CommBase           base;      // 基类
  I2C_HandleTypeDef  *hi2c;     // HAL I2C 句柄
  uint8_t             dev_addr; // 7位设备地址（已左移1位，即 HAL 需要的格式）
} Iic;

void iic_init(Iic *me, CommName name, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);
void iic_deinit(Iic *me);

// 扩展 API: 写寄存器地址后读 N 字节 —— 传感器访问标准模式
// 用法: iic_read_reg(&i2c_dev, 0x3B, buf, 6); // 读 MPU6050 加速度计
uint8_t iic_read_reg(Iic *me, uint8_t reg, uint8_t *dst, uint8_t len);

#endif
