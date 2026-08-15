// MPU6050 平台层 API —— 芯片能力声明
//
// 迁移: 原 Components/sensor/comp_mpu.h (阶段2 peripheral 域收编)
//   本文件同时承载 DMP 固件 (comp_mpu_dmp.c) 的 4 个传输接缝声明,
//   实现在 Devices/peripheral 的 per_mpu6050.c (转发单例 Iic*).
//
// 返回 0 = 成功, 非 0 = 失败 (read_byte 除外: 返回寄存器值)

#ifndef COMP_MPU_H
#define COMP_MPU_H

#include <stdint.h>

// ============================================================
// 传输接缝 (per_mpu6050.c 实现, 转发单例 Iic*) — DMP 固件调用点
// 名字与 Devices/sensor/mpu6050.h 保持一致 → 固件零改动
// ============================================================
uint8_t mpu6050_write_byte(uint8_t reg, uint8_t data);                    // 写 1 字节, 返回 0=成功
uint8_t mpu6050_read_byte(uint8_t reg);                                   // 读 1 字节, 返回寄存器值 (失败返回 0)
uint8_t mpu6050_write_len(uint8_t reg, uint8_t len, const uint8_t *buf);  // 写 len 字节, 返回 0=成功
uint8_t mpu6050_read_len(uint8_t reg, uint8_t len, uint8_t *buf);         // 读 len 字节, 返回 0=成功

// ============================================================
// 基础芯片能力 (plat_mpu.c 实现)
// ============================================================

// 初始化: 复位 → 唤醒 → PLL → LPF=42Hz → FSR=±2000dps/±2g → 50Hz
int mpu_init(void);

// 传感器使能 (INV_X_GYRO|INV_Y_GYRO|INV_Z_GYRO = 0x70, INV_XYZ_ACCEL = 0x08)
int mpu_set_sensors(uint8_t sensors);

// FIFO 配置 (指定哪些传感器数据进入 FIFO)
int mpu_configure_fifo(uint8_t sensors);

// 采样率 4~1000 Hz (自动匹配 LPF = 采样率/2)
int mpu_set_sample_rate(uint16_t rate);

// 量程: 0=±250, 1=±500, 2=±1000, 3=±2000 dps
int mpu_set_gyro_fsr(uint8_t fsr);

// 量程: 0=±2g, 1=±4g, 2=±8g, 3=±16g
int mpu_set_accel_fsr(uint8_t fsr);

// 数字低通滤波: 188, 98, 42, 20, 10, 5 Hz
int mpu_set_lpf(uint16_t lpf);

// 中断使能 (写 INT_ENABLE 寄存器, DMP 模式下自动切为 DMP 中断)
void mpu_set_int_enable(uint8_t enable);

// 启动/停止 DMP (写 USER_CTRL 寄存器)
int mpu_set_dmp_state(uint8_t enable);

// 复位 FIFO
int mpu_reset_fifo(void);

// 直接读传感器寄存器 (绕过 DMP, 用于回退模式)
int mpu_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);
int mpu_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
int mpu_read_temp(int16_t *temp);

// 仅读陀螺仪 Z 轴 (2 字节, 高速 ~200us)
int16_t mpu_read_gyro_z(void);

// 器件检测 (读 WHO_AM_I, 返回 0 = 正确)
int mpu_check_id(void);

// ============================================================
// DMP 芯片能力 (comp_mpu_dmp.c 实现)
// ============================================================

// DMP 初始化: 加载固件 → 配置特性 → 自检 → 启动
int mpu_dmp_init(void);

// 读一个 DMP FIFO 包, 同时返回欧拉角 + 校准后陀螺仪值
// pitch/roll/yaw: 度 (DMP 四元数硬件解算, 无漂移)
// gx/gy/gz: 陀螺仪原始值 (DMP 已校准)
int mpu_dmp_read_fifo(float *pitch, float *roll, float *yaw, int16_t *gx, int16_t *gy, int16_t *gz);

#endif
