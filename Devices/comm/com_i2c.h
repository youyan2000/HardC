// I2C 传输类 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 语义: 从机寻址 + 寄存器访问. 双模式:
//   IIC_HW — 硬件 I2C (HAL_I2C_Mem_Write/Mem_Read, 100ms 超时)
//   IIC_SW — 软件 bit-bang (纯 C 走 bsp_gpio, SCL/SDA 引脚注入, host 可测)
//
// 数据面 (模式/句柄/地址/引脚) 在子类结构体, 不进 CommBase 虚表;
// 保留 iic_read_reg 语义 (写寄存器地址→读 N 字节, 传感器访问标准模式).

#ifndef COM_I2C_H
#define COM_I2C_H

#include "comp_comm.h"
#include "comp_io.h"
#include "comp_error_code.h"
#include "bsp_gpio.h"
#include "bsp_stm32_hal.h"

typedef enum { IIC_HW = 0, IIC_SW = 1 } IicMode;

// I2C 类 — HW (HAL I2C) + SW (bit-bang 走 bsp_gpio)
typedef struct {
  CommBase base;            // 基类 (必须第一成员)
  I2C_HandleTypeDef *hi2c;  // HW 模式句柄 (iic_init_hw)
  uint8_t dev_addr;         // 7-bit 从机地址
  BspGpioPin scl;           // SW 模式时钟脚
  BspGpioPin sda;           // SW 模式数据脚
  IicMode mode;             // 当前模式
  IoCompletion completion;
} Iic;

// HW 模式初始化
void iic_init_hw(Iic *me, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);

// SW 模式初始化 (bit-bang)
void iic_init_sw(Iic *me, BspGpioPin scl, BspGpioPin sda, uint8_t dev_addr);

void iic_deinit(Iic *me);

// 写寄存器地址 → 写 len 字节 (传感器寄存器访问模式)
ErrorCode iic_write_reg(Iic *me, uint8_t reg, const uint8_t *dat, uint16_t len, IoCompletion comp);

// 写寄存器地址 → 读 len 字节 (传感器寄存器访问模式)
ErrorCode iic_read_reg(Iic *me, uint8_t reg, uint8_t *dat, uint16_t len, IoCompletion comp);

#endif  // COM_I2C_H
