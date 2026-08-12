// MPU6050 BSP 硬件层头文件 —— 软件 I2C bit-bang 接口声明
// 源自: LitteCar 项目 (SmCar/LitteCar_STM32/c8t6_car_24H/User/BSP/mpu6050.h)

#ifndef BSP_MPU6050_H
#define BSP_MPU6050_H

// MPU6050 硬件操作层 —— 软件 I2C bit-bang + 寄存器读写
// SCL=PB14, SDA=PB15, AD0=GND → 器件地址 0x68
// 只依赖 <stdint.h>, 不依赖 STM32 HAL, 换 MCU 只需替换 .c 实现

#include <stdint.h>

// I2C 协议函数
void     i2c_init(void);
void     i2c_start(void);
void     i2c_stop(void);
void     i2c_send_byte(uint8_t txd);
uint8_t  i2c_read_byte(uint8_t ack);
uint8_t  i2c_wait_ack(void);
void     i2c_ack(void);
void     i2c_nack(void);

// MPU6050 寄存器操作
uint8_t  mpu6050_write_byte(uint8_t reg, uint8_t data);
uint8_t  mpu6050_read_byte (uint8_t reg);
uint8_t  mpu6050_write_len (uint8_t reg, uint8_t len, const uint8_t *buf);
uint8_t  mpu6050_read_len  (uint8_t reg, uint8_t len, uint8_t *buf);

// MPU6050 初始化和配置
uint8_t  mpu6050_device_init(void);
uint8_t  mpu6050_set_gyro_fsr(uint8_t fsr);
uint8_t  mpu6050_set_accel_fsr(uint8_t fsr);
uint8_t  mpu6050_set_rate(uint16_t rate);

// 传感器数据读取
short    mpu6050_get_temp(void);
uint8_t  mpu6050_get_gyro(short *gx, short *gy, short *gz);
uint8_t  mpu6050_get_accel(short *ax, short *ay, short *az);
short    mpu6050_get_gyro_z(void);   // 仅读 Z 轴 (快速, ~200us)

// 陀螺仪量程 ±2000dps → 灵敏度 16.4 LSB/(°/s)
#define MPU6050_GYRO_SCALE  16.384f

// 软件延时 (定义在 bsp_delay.c, 此处仅声明供硬件层使用)
void Delay_ms(uint32_t nms);
void Delay_us(uint32_t nus);

#endif
