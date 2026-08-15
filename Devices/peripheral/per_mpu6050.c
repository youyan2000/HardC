// MPU6050 六轴 IMU 外设 — 对象契约 + 全局传输接缝实现
// 迁移自 Devices/sensor/mpu6050.c, 底层从软件 I2C bit-bang 改为 Iic 类总线转发

#include "per_mpu6050.h"
#include "comp_mpu.h"
#include "bsp_delay.h"
#include "container_of.h"
#include <stddef.h>

// MPU6050 从机地址 (7-bit) — 器件 ID 校验用, 总线 dev_addr 由 App 注入
#define MPU_ADDR 0x68

// 单例 — DMP 固件接缝转发目标 (per_mpu6050_init 登记)
static PerMpu6050 *s_mpu;

// MPU6050 寄存器地址
#define REG_PWR_MGMT1 0x6B
#define REG_PWR_MGMT2 0x6C
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CFG 0x1B
#define REG_ACCEL_CFG 0x1C
#define REG_INT_EN 0x38
#define REG_USER_CTRL 0x6A
#define REG_FIFO_EN 0x23
#define REG_INTBP_CFG 0x37
#define REG_TEMP_OUT 0x41
#define REG_GYRO_XOUT 0x43
#define REG_ACCEL_XOUT 0x3B
#define REG_DEVICE_ID 0x75

// -------- 传输接缝 (comp_mpu.h 声明, DMP 固件调用点) --------

// 写 1 字节寄存器 — 转发单例总线 iic_write_reg, 返回 0=成功
uint8_t mpu6050_write_byte(uint8_t reg, uint8_t data) {
  if (s_mpu == NULL || s_mpu->bus == NULL) {
    return 1;
  }
  return iic_write_reg(s_mpu->bus, reg, &data, 1, IO_SYNC) == ERR_OK ? 0 : 1;
}

// 读 1 字节寄存器 — 转发单例总线 iic_read_reg, 返回寄存器值 (失败返回 0)
uint8_t mpu6050_read_byte(uint8_t reg) {
  uint8_t v = 0;
  if (s_mpu == NULL || s_mpu->bus == NULL) {
    return 0;
  }
  if (iic_read_reg(s_mpu->bus, reg, &v, 1, IO_SYNC) != ERR_OK) {
    return 0;
  }
  return v;
}

// 写 len 字节寄存器 — 转发单例总线 iic_write_reg, 返回 0=成功
uint8_t mpu6050_write_len(uint8_t reg, uint8_t len, const uint8_t *buf) {
  if (s_mpu == NULL || s_mpu->bus == NULL) {
    return 1;
  }
  return iic_write_reg(s_mpu->bus, reg, buf, len, IO_SYNC) == ERR_OK ? 0 : 1;
}

// 读 len 字节寄存器 — 转发单例总线 iic_read_reg, 返回 0=成功
uint8_t mpu6050_read_len(uint8_t reg, uint8_t len, uint8_t *buf) {
  if (s_mpu == NULL || s_mpu->bus == NULL) {
    return 1;
  }
  return iic_read_reg(s_mpu->bus, reg, buf, len, IO_SYNC) == ERR_OK ? 0 : 1;
}

// -------- 基础寄存器配置 (原 mpu6050.c 迁移, 底层改走接缝) --------

// 配置陀螺仪量程: 0=±250, 1=±500, 2=±1000, 3=±2000 dps
static uint8_t mpu6050_set_gyro_fsr(uint8_t fsr) {
  return mpu6050_write_byte(REG_GYRO_CFG, (uint8_t) (fsr << 3));
}

// 配置加速度量程: 0=±2g, 1=±4g, 2=±8g, 3=±16g
static uint8_t mpu6050_set_accel_fsr(uint8_t fsr) {
  return mpu6050_write_byte(REG_ACCEL_CFG, (uint8_t) (fsr << 3));
}

// 配置采样率 + 数字低通 (按 eMPL 逻辑 DLPF_CFG 匹配采样率/2)
static uint8_t mpu6050_set_rate(uint16_t rate) {
  uint8_t data, lpf;
  if (rate > 1000) {
    rate = 1000;
  }
  if (rate < 4) {
    rate = 4;
  }
  data = (uint8_t) (1000 / rate - 1);
  data = mpu6050_write_byte(REG_SMPLRT_DIV, data);
  lpf = (uint8_t) (rate >> 1);
  if (lpf >= 188) {
    data = 1;  // DLPF_CFG=1: 184Hz
  } else if (lpf >= 98) {
    data = 2;  // DLPF_CFG=2:  94Hz
  } else if (lpf >= 42) {
    data = 3;  // DLPF_CFG=3:  44Hz
  } else if (lpf >= 20) {
    data = 4;  // DLPF_CFG=4:  21Hz
  } else if (lpf >= 10) {
    data = 5;  // DLPF_CFG=5:  10Hz
  } else {
    data = 6;  // DLPF_CFG=6:   5Hz
  }
  return mpu6050_write_byte(REG_CONFIG, data);
}

// 器件初始化: 复位 → 唤醒 → 量程 → 采样率 → 校验 ID → PLL
static uint8_t mpu6050_device_init(void) {
  uint8_t id;
  mpu6050_write_byte(REG_PWR_MGMT1, 0x80);  // 复位
  Delay_ms(100);
  mpu6050_write_byte(REG_PWR_MGMT1, 0x00);  // 唤醒
  mpu6050_set_gyro_fsr(3);                  // ±2000dps
  mpu6050_set_accel_fsr(0);                 // ±2g
  mpu6050_set_rate(100);                    // 100Hz
  mpu6050_write_byte(REG_INT_EN, 0x00);
  mpu6050_write_byte(REG_USER_CTRL, 0x00);
  mpu6050_write_byte(REG_FIFO_EN, 0x00);
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

// -------- 传感器数据读取 (原 mpu6050.c 迁移) --------

// 读温度 (x100)
static int16_t mpu6050_get_temp(void) {
  uint8_t buf[2];
  int16_t raw;
  if (mpu6050_read_len(REG_TEMP_OUT, 2, buf) != 0) {
    return 0;
  }
  raw = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
  return (int16_t) ((36.53f + (float) raw / 340.0f) * 100.0f);
}

// 读陀螺仪原始值
static uint8_t mpu6050_get_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
  uint8_t buf[6];
  uint8_t res = mpu6050_read_len(REG_GYRO_XOUT, 6, buf);
  if (res == 0) {
    *gx = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
    *gy = (int16_t) (((uint16_t) buf[2] << 8) | buf[3]);
    *gz = (int16_t) (((uint16_t) buf[4] << 8) | buf[5]);
  }
  return res;
}

// 读加速度原始值
static uint8_t mpu6050_get_accel(int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t buf[6];
  uint8_t res = mpu6050_read_len(REG_ACCEL_XOUT, 6, buf);
  if (res == 0) {
    *ax = (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
    *ay = (int16_t) (((uint16_t) buf[2] << 8) | buf[3]);
    *az = (int16_t) (((uint16_t) buf[4] << 8) | buf[5]);
  }
  return res;
}

// 仅读陀螺仪 Z 轴 (2 字节), 转弯角度积分用
static int16_t mpu6050_get_gyro_z(void) {
  uint8_t buf[2];
  if (mpu6050_read_len((uint8_t) (REG_GYRO_XOUT + 4), 2, buf) == 0) {
    return (int16_t) (((uint16_t) buf[0] << 8) | buf[1]);
  }
  return 0;
}

// -------- 对象契约 --------

// measure 分发 — container_of 下溯到 PerMpu6050
static int mpu_measure_impl(SensorBase *base) {
  PerMpu6050 *me = container_of(base, PerMpu6050, base);
  return per_mpu6050_measure(me);
}

// 初始化: 绑总线 + 设单例 → 基础寄存器配置 → 绑 measure
int per_mpu6050_init(PerMpu6050 *me, Iic *bus) {
  sensor_base_init(&me->base, "mpu6050");
  me->bus = bus;
  me->gyro[0] = 0;
  me->gyro[1] = 0;
  me->gyro[2] = 0;
  me->accel[0] = 0;
  me->accel[1] = 0;
  me->accel[2] = 0;
  me->temp = 0;
  me->pitch = 0.0f;
  me->roll = 0.0f;
  me->yaw = 0.0f;
  me->dmp_enabled = 0;
  me->valid = 0;
  s_mpu = me;
  me->base.measure = mpu_measure_impl;

  if (mpu6050_device_init() != 0) {
    me->base.inited = 0;
    return ERR_FAILED;
  }
  me->base.inited = 1;
  return ERR_OK;
}

// [可选] 启动 DMP: 固件初始化 (内部调 comp_mpu.h 平台层 mpu_set_* 等)
int per_mpu6050_dmp_start(PerMpu6050 *me) {
  if (mpu_dmp_init() == 0) {
    me->dmp_enabled = 1;
    return ERR_OK;
  }
  return ERR_FAILED;
}

// 触发一次测量: DMP → FIFO 读欧拉角+陀螺仪; 否则寄存器回退
int per_mpu6050_measure(PerMpu6050 *me) {
  if (me->base.inited == 0) {
    return ERR_STATE;
  }

  if (me->dmp_enabled) {
    int16_t gx, gy, gz;
    if (mpu_dmp_read_fifo(&me->pitch, &me->roll, &me->yaw, &gx, &gy, &gz) == 0) {
      me->gyro[0] = gx;
      me->gyro[1] = gy;
      me->gyro[2] = gz;
      mpu6050_get_accel(&me->accel[0], &me->accel[1], &me->accel[2]);
      me->temp = mpu6050_get_temp();
      me->valid = 1;
      return ERR_OK;
    }
    me->valid = 0;
    return ERR_EMPTY;  // FIFO 暂无可读包, 保持上次姿态
  }

  // 寄存器回退
  if (mpu6050_get_gyro(&me->gyro[0], &me->gyro[1], &me->gyro[2]) == 0 &&
      mpu6050_get_accel(&me->accel[0], &me->accel[1], &me->accel[2]) == 0) {
    me->temp = mpu6050_get_temp();
    me->valid = 1;
    return ERR_OK;
  }
  me->valid = 0;
  return ERR_FAILED;
}

// 仅读陀螺仪 Z 轴 (2 字节, ~200us) — 高速轮询, 转弯角度积分用
int16_t per_mpu6050_read_gyro_z(PerMpu6050 *me) {
  (void) me;
  return mpu6050_get_gyro_z();
}

// -------- plat 芯片能力 (comp_mpu.h 声明, 全局函数经 s_mpu 单例走接缝) --------
// DMP 固件 (comp_mpu_dmp.c) 直接引用这 8 个函数, 不实现则 per_mpu6050_dmp_start 链接失败.
// 全部写寄存器操作, 复用上方静态基础配置 + 4 个传输接缝.

// 传感器掩码 (与 comp_mpu_dmp.c 内部 INV_* 定义保持一致)
#define INV_X_GYRO (0x40)
#define INV_Y_GYRO (0x20)
#define INV_Z_GYRO (0x10)
#define INV_XYZ_ACCEL (0x08)

// USER_CTRL 控制位 (与 comp_mpu_dmp.c 的 BIT_DMP_RST/BIT_FIFO_RST 同源)
#define MPU_DMP_EN (0x80)
#define MPU_FIFO_RST (0x04)

// 传感器使能: PWR_MGMT_1 PLL 时钟唤醒 + PWR_MGMT_2 使能对应轴
int mpu_set_sensors(uint8_t sensors) {
  uint8_t pwr2 = 0x7E;  // PWR_MGMT_2 默认全禁用 (bit6:4 accel, bit3:1 gyro)
  if (sensors & INV_XYZ_ACCEL) {
    pwr2 &= (uint8_t) ~0x70;  // 使能加速度计
  }
  if (sensors & INV_X_GYRO) {
    pwr2 &= (uint8_t) ~0x08;  // 使能陀螺 X
  }
  if (sensors & INV_Y_GYRO) {
    pwr2 &= (uint8_t) ~0x04;  // 使能陀螺 Y
  }
  if (sensors & INV_Z_GYRO) {
    pwr2 &= (uint8_t) ~0x02;  // 使能陀螺 Z
  }
  mpu6050_write_byte(REG_PWR_MGMT2, pwr2);
  return mpu6050_write_byte(REG_PWR_MGMT1, 0x01);  // PLL X 轴时钟源唤醒
}

// FIFO 配置: FIFO_EN 传感器掩码 (bit5:3 陀螺, bit2 加速度计)
int mpu_configure_fifo(uint8_t sensors) {
  uint8_t data = 0;
  if (sensors & INV_X_GYRO) {
    data |= 0x20;
  }
  if (sensors & INV_Y_GYRO) {
    data |= 0x10;
  }
  if (sensors & INV_Z_GYRO) {
    data |= 0x08;
  }
  if (sensors & INV_XYZ_ACCEL) {
    data |= 0x04;
  }
  return mpu6050_write_byte(REG_FIFO_EN, data);
}

// 采样率 4~1000Hz: 复用静态 mpu6050_set_rate (SMPLRT_DIV + 匹配 LPF)
int mpu_set_sample_rate(uint16_t rate) {
  return mpu6050_set_rate(rate);
}

// 陀螺仪量程: 0/1/2/3 → ±250/500/1000/2000 dps
int mpu_set_gyro_fsr(uint8_t fsr) {
  return mpu6050_set_gyro_fsr(fsr);
}

// 加速度量程: 0/1/2/3 → ±2/4/8/16g
int mpu_set_accel_fsr(uint8_t fsr) {
  return mpu6050_set_accel_fsr(fsr);
}

// 数字低通: CONFIG 写 DLPF_CFG (MPU6050 DLPF 表: 184/94/44/21/10/5 Hz)
int mpu_set_lpf(uint16_t lpf) {
  uint8_t dlpf;
  if (lpf >= 188) {
    dlpf = 1;
  } else if (lpf >= 98) {
    dlpf = 2;
  } else if (lpf >= 42) {
    dlpf = 3;
  } else if (lpf >= 20) {
    dlpf = 4;
  } else if (lpf >= 10) {
    dlpf = 5;
  } else {
    dlpf = 6;
  }
  return mpu6050_write_byte(REG_CONFIG, dlpf);
}

// 启动/停止 DMP: USER_CTRL 置/清 DMP_EN 位, 启动时复位 FIFO
int mpu_set_dmp_state(uint8_t enable) {
  uint8_t uc = mpu6050_read_byte(REG_USER_CTRL);
  if (enable) {
    uc |= MPU_DMP_EN;
    mpu6050_write_byte(REG_USER_CTRL, uc);
    return mpu_reset_fifo();
  }
  uc &= (uint8_t) ~MPU_DMP_EN;
  return mpu6050_write_byte(REG_USER_CTRL, uc);
}

// 复位 FIFO: USER_CTRL FIFO_RST 位置位再清零
int mpu_reset_fifo(void) {
  uint8_t uc = mpu6050_read_byte(REG_USER_CTRL);
  mpu6050_write_byte(REG_USER_CTRL, (uint8_t) (uc | MPU_FIFO_RST));
  return mpu6050_write_byte(REG_USER_CTRL, (uint8_t) (uc & (uint8_t) ~MPU_FIFO_RST));
}
