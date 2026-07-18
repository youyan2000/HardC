// MPU6050 硬件操作层 —— 软件 I2C bit-bang + 寄存器操作
// SCL=PB14, SDA=PB15

#include "mpu6050.h"
#include "bsp_delay.h"

#define I2C_DELAY_US  2

// MPU6050 寄存器地址
#define REG_PWR_MGMT1   0x6B
#define REG_PWR_MGMT2   0x6C
#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_INT_EN      0x38
#define REG_USER_CTRL   0x6A
#define REG_FIFO_EN     0x23
#define REG_INTBP_CFG   0x37
#define REG_TEMP_OUT    0x41
#define REG_GYRO_XOUT   0x43
#define REG_ACCEL_XOUT  0x3B
#define REG_DEVICE_ID   0x75

#define MPU_ADDR        0x68

// -------- SDA/SCL 方向切换 --------

static void sda_output(void) {
 GPIO_InitTypeDef s = { .Pin = I2C_SDA_Pin, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_LOW };
 HAL_GPIO_Init(I2C_SDA_GPIO_Port, &s);
}

static void sda_input(void) {
 GPIO_InitTypeDef s = { .Pin = I2C_SDA_Pin, .Mode = GPIO_MODE_INPUT, .Speed = GPIO_SPEED_FREQ_LOW };
 HAL_GPIO_Init(I2C_SDA_GPIO_Port, &s);
}

static void scl_output(void) {
 GPIO_InitTypeDef s = { .Pin = I2C_SCL_Pin, .Mode = GPIO_MODE_OUTPUT_PP, .Speed = GPIO_SPEED_FREQ_LOW };
 HAL_GPIO_Init(I2C_SCL_GPIO_Port, &s);
}

// -------- I2C 协议 --------

void i2c_init(void) {
 scl_output();
 sda_output();
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
}

void i2c_start(void) {
 sda_output();
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
}

void i2c_stop(void) {
 sda_output();
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
}

uint8_t i2c_wait_ack(void) {
 uint8_t t = 0;
 sda_input();
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 while (HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)) {
  if (++t > 250) { i2c_stop(); return 1; }
 }
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 return 0;
}

void i2c_ack(void) {
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 sda_output();
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
}

void i2c_nack(void) {
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 sda_output();
 HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
 Delay_us(I2C_DELAY_US);
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
}

void i2c_send_byte(uint8_t txd) {
 sda_output();
 HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 for (uint8_t t = 0; t < 8; t++) {
  HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, (txd & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  txd <<= 1;
  Delay_us(I2C_DELAY_US);
  HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
  Delay_us(I2C_DELAY_US);
  HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
 }
}

uint8_t i2c_read_byte(uint8_t ack) {
 uint8_t r = 0;
 sda_input();
 for (uint8_t i = 0; i < 8; i++) {
  HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET);
  Delay_us(I2C_DELAY_US);
  HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET);
  r <<= 1;
  if (HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin)) r++;
  Delay_us(I2C_DELAY_US);
 }
 ack ? i2c_ack() : i2c_nack();
 return r;
}

// -------- MPU6050 寄存器读写 --------

uint8_t mpu6050_write_byte(uint8_t reg, uint8_t data) {
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 0);
 if (i2c_wait_ack()) { i2c_stop(); return 1; }
 i2c_send_byte(reg);
 i2c_wait_ack();
 i2c_send_byte(data);
 if (i2c_wait_ack()) { i2c_stop(); return 1; }
 i2c_stop();
 return 0;
}

uint8_t mpu6050_read_byte(uint8_t reg) {
 uint8_t res;
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 0);
 if (i2c_wait_ack()) { i2c_stop(); return 0; }
 i2c_send_byte(reg);
 i2c_wait_ack();
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 1);
 i2c_wait_ack();
 res = i2c_read_byte(0);
 i2c_stop();
 return res;
}

uint8_t mpu6050_write_len(uint8_t reg, uint8_t len, const uint8_t *buf) {
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 0);
 if (i2c_wait_ack()) { i2c_stop(); return 1; }
 i2c_send_byte(reg);
 i2c_wait_ack();
 for (uint8_t i = 0; i < len; i++) {
  i2c_send_byte(buf[i]);
  if (i2c_wait_ack()) { i2c_stop(); return 1; }
 }
 i2c_stop();
 return 0;
}

uint8_t mpu6050_read_len(uint8_t reg, uint8_t len, uint8_t *buf) {
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 0);
 if (i2c_wait_ack()) { i2c_stop(); return 1; }
 i2c_send_byte(reg);
 i2c_wait_ack();
 i2c_start();
 i2c_send_byte((MPU_ADDR << 1) | 1);
 i2c_wait_ack();
 while (len) {
  *buf = i2c_read_byte(len > 1);
  len--;
  buf++;
 }
 i2c_stop();
 return 0;
}

// -------- MPU6050 初始化 --------

uint8_t mpu6050_device_init(void) {
 uint8_t id;
 i2c_init();
 mpu6050_write_byte(REG_PWR_MGMT1, 0x80);  // 复位
 Delay_ms(100);
 mpu6050_write_byte(REG_PWR_MGMT1, 0x00);  // 唤醒
 mpu6050_set_gyro_fsr(3);                  // ±2000dps
 mpu6050_set_accel_fsr(0);                 // ±2g
 mpu6050_set_rate(100);                    // 100Hz
 mpu6050_write_byte(REG_INT_EN,    0x00);
 mpu6050_write_byte(REG_USER_CTRL, 0x00);
 mpu6050_write_byte(REG_FIFO_EN,   0x00);
 mpu6050_write_byte(REG_INTBP_CFG, 0x80);
 id = mpu6050_read_byte(REG_DEVICE_ID);
 if (id == MPU_ADDR) {
  mpu6050_write_byte(REG_PWR_MGMT1, 0x01);  // PLL X 轴
  mpu6050_write_byte(REG_PWR_MGMT2, 0x00);  // 陀螺仪+加速度计都工作
  mpu6050_set_rate(100);                    // 切换 PLL 后重新确认采样率
  return 0;
 }
 return 1;
}

// -------- 传感器配置 --------

uint8_t mpu6050_set_gyro_fsr(uint8_t fsr) {
 return mpu6050_write_byte(REG_GYRO_CFG, fsr << 3);
}

uint8_t mpu6050_set_accel_fsr(uint8_t fsr) {
 return mpu6050_write_byte(REG_ACCEL_CFG, fsr << 3);
}

uint8_t mpu6050_set_rate(uint16_t rate) {
 uint8_t data, lpf;
 if (rate > 1000) rate = 1000;
 if (rate < 4)    rate = 4;
 data = 1000 / rate - 1;
 data = mpu6050_write_byte(REG_SMPLRT_DIV, data);
 // 按参考项目 eMPL 逻辑: LPF = 采样率/2, 按要求匹配到 DLPF_CFG
 lpf = rate >> 1;
 if      (lpf >= 188) data = 1;   // DLPF_CFG=1: 184Hz
 else if (lpf >= 98)  data = 2;   // DLPF_CFG=2:  94Hz
 else if (lpf >= 42)  data = 3;   // DLPF_CFG=3:  44Hz
 else if (lpf >= 20)  data = 4;   // DLPF_CFG=4:  21Hz
 else if (lpf >= 10)  data = 5;   // DLPF_CFG=5:  10Hz
 else                 data = 6;   // DLPF_CFG=6:   5Hz
 return mpu6050_write_byte(REG_CONFIG, data);
}

// -------- 传感器数据读取 --------

short mpu6050_get_temp(void) {
 uint8_t buf[2];
 short raw;
 mpu6050_read_len(REG_TEMP_OUT, 2, buf);
 raw = ((uint16_t)buf[0] << 8) | buf[1];
 return (short)(36.53f + (float)raw / 340.0f) * 100;
}

uint8_t mpu6050_get_gyro(short *gx, short *gy, short *gz) {
 uint8_t buf[6], res;
 res = mpu6050_read_len(REG_GYRO_XOUT, 6, buf);
 if (res == 0) {
  *gx = ((uint16_t)buf[0] << 8) | buf[1];
  *gy = ((uint16_t)buf[2] << 8) | buf[3];
  *gz = ((uint16_t)buf[4] << 8) | buf[5];
 }
 return res;
}

uint8_t mpu6050_get_accel(short *ax, short *ay, short *az) {
 uint8_t buf[6], res;
 res = mpu6050_read_len(REG_ACCEL_XOUT, 6, buf);
 if (res == 0) {
  *ax = ((uint16_t)buf[0] << 8) | buf[1];
  *ay = ((uint16_t)buf[2] << 8) | buf[3];
  *az = ((uint16_t)buf[4] << 8) | buf[5];
 }
 return res;
}

// 仅读陀螺仪 Z 轴 (2 字节), 用于转弯角度积分, 比读 6 字节快 3 倍
short mpu6050_get_gyro_z(void) {
 uint8_t buf[2];
 if (mpu6050_read_len(REG_GYRO_XOUT + 4, 2, buf) == 0) {
  return ((uint16_t)buf[0] << 8) | buf[1];
 }
 return 0;
}
