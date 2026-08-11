#ifndef COM_MPU6050_H
#define COM_MPU6050_H

// MPU6050 通信驱动 —— CommBase 子类, 只调平台层 plat_mpu.h API
// DMP 模式: 姿态由硬件四元数融合 (无漂移), 通过 INT 引脚触发 ISR 读取
// 回退模式: 软件 atan2 + 陀螺仪积分 (原行为)
// send: I2C 写寄存器  bgn/read: 空操作

#include "comp_comm.h"

typedef struct Mpu6050 {
  CommBase base;            // 基类 (必须第一个)
  uint8_t  initialized;     // 初始化成功标志
  uint8_t  dmp_enabled;     // DMP 是否已启动 (1=硬件姿态, 0=软件回退)
  short    gx, gy, gz;      // 陀螺仪原始值
  short    ax, ay, az;      // 加速度原始值
  short    temp;            // 温度值 (x100)
  float    roll;            // 横滚角 (度)
  float    pitch;           // 俯仰角 (度)
  float    yaw;             // 偏航角速度 (dps, 陀螺仪 Z 轴)
  float    yaw_abs;         // 绝对偏航角 (度, EMA 滤波积分, 供显示/长期参考)
  float    yaw_fast;        // 绝对偏航角 (度, 原始值积分, 零滞后, 供转弯控制器)
  float    gyro_z_raw;      // 陀螺仪 Z 角速度 deg/s (DMP: 校准后; 回退: 原始-零偏)
  float    gyro_z_offset;   // 陀螺仪 Z 轴零偏 (init 时 50 次采样均值, 仅回退模式)
  float    gz_filtered;     // 陀螺仪 Z 低通滤波值 (EMA, 抑制电机振动/颠簸)

  volatile uint8_t dmp_data_ready; // ISR 置 1, 消费后清 0
} Mpu6050;

void mpu6050_drv_init(Mpu6050 *me, CommName name);

// 主循环高速轮询陀螺仪 Z (仅回退模式, 避免 ISR 中软 I2C)
void mpu6050_drv_poll_gyro_z(Mpu6050 *me);

// 读取所有传感器数据并更新内部缓存
void mpu6050_read_all(Mpu6050 *me);

// 每 10ms 调用 (ISR), 连续积分 yaw
void mpu6050_yaw_integrate(Mpu6050 *me);

// DMP INT 中断服务 (在 EXTI 回调中调用, 读 FIFO 并缓存姿态)
void mpu6050_drv_dmp_isr(Mpu6050 *me);

#endif
