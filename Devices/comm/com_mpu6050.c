// MPU6050 通信驱动 —— CommBase 子类, 只调平台层 plat_mpu.h API
// DMP 模式: 姿态由硬件四元数融合 (无漂移), INT 触发 ISR 读 FIFO → 缓存
// 回退模式: 软件 atan2 + 陀螺仪积分 (原行为, 有漂移)

#include "com_mpu6050.h"
#include "comp_mpu.h"
#include "container_of.h"
#include <math.h>

// 陀螺仪 ±2000dps → 灵敏度 16.4 LSB/(°/s)
#define GYRO_SCALE  16.384f

// -------- ops 实现 --------
// MPU6050 通过 I2C 直接读写, CommBase::send 不用于此设备
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  (void)base; (void)dat; (void)len;
}

// MPU6050 用 I2C 直接读写, 不通过 CommBase 中断接收
static void bgn_impl(CommBase *base) {
  (void)base;
}

// 当前字节无意义 (I2C 返回数据不缓存在 base.cur)
static uint8_t read_impl(CommBase *base) {
  (void)base;
  return 0;
}

static const CommOps mpu6050_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 --------

void mpu6050_drv_init(Mpu6050 *me, CommName name) {
  comm_base_init(&me->base);
  me->base.name  = name;
  me->base.ops   = &mpu6050_ops;
  me->initialized = 0;
  me->dmp_enabled = 0;
  me->dmp_data_ready = 0;
  me->gx = me->gy = me->gz = 0;
  me->ax = me->ay = me->az = 0;
  me->temp = 0;
  me->roll  = 0;
  me->pitch = 0;
  me->yaw   = 0;
  me->yaw_abs = 0;
  me->yaw_fast = 0;
  me->gz_filtered = 0;
  me->gyro_z_offset = 0;
  me->gyro_z_raw = 0;

  // 1. 平台层基础初始化 (复位 → 唤醒 → PLL → LPF=42Hz → FSR=±2000dps/±2g)
  if (mpu_init() != 0) return;
  me->initialized = 1;

  // 2. 尝试 DMP 初始化 (硬件四元数融合, 无漂移)
  if (mpu_dmp_init() == 0) {
    me->dmp_enabled = 1;
    return;
  }

  // 3. 回退: 陀螺仪 Z 轴零偏校准 (静止状态采 50 次取均值)
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += mpu_read_gyro_z();
    mpu_delay_ms(2);
  }
  me->gyro_z_offset = (float)sum / 50.0f;
}

// -------- 主循环高速轮询陀螺仪 Z (仅回退模式, 避免 ISR 中软 I2C) --------

void mpu6050_drv_poll_gyro_z(Mpu6050 *me) {
  if (me->initialized && !me->dmp_enabled) {
    me->gz = mpu_read_gyro_z();
  }
}

// -------- 读取所有传感器数据 --------

#define RAD_TO_DEG  57.29578f

void mpu6050_read_all(Mpu6050 *me) {
  if (!me->initialized) return;

  if (me->dmp_enabled) {
    // DMP 模式: pitch/roll/yaw 已由 ISR 更新, 只补读加速度计和温度
    mpu_read_accel(&me->ax, &me->ay, &me->az);
    mpu_read_temp(&me->temp);
  } else {
    // 回退模式: 软件 atan2 + 陀螺仪直接读
    mpu_read_gyro(&me->gx, &me->gy, &me->gz);
    mpu_read_accel(&me->ax, &me->ay, &me->az);
    mpu_read_temp(&me->temp);
    me->roll  = atan2f((float)me->ay, (float)me->az) * RAD_TO_DEG;
    me->pitch = atan2f((float)(-me->ax),
      sqrtf((float)me->ay * me->ay + (float)me->az * me->az)) * RAD_TO_DEG;
    me->gyro_z_raw = ((float)me->gz - me->gyro_z_offset) / GYRO_SCALE;
    me->yaw = me->gyro_z_raw;
  }
}

// 每 10ms 调用 (ISR), 连续积分绝对偏航角
void mpu6050_yaw_integrate(Mpu6050 *me) {
  if (!me->initialized) return;

  if (me->dmp_enabled) {
    // DMP 模式: gyro_z_raw 已由 ISR 从 FIFO 校准值计算, 直接积分
    float rate = me->gyro_z_raw;
    me->gz_filtered = me->gz_filtered * 0.25f + rate * 0.75f;
    me->yaw_fast += rate * 0.01f;
  } else {
    // 回退模式: 原始值减零偏 → 积分
    float rate = ((float)me->gz - me->gyro_z_offset) / GYRO_SCALE;
    me->gz_filtered = me->gz_filtered * 0.25f + (me->gz - me->gyro_z_offset) * 0.75f;
    me->yaw_abs  += me->gz_filtered / GYRO_SCALE * 0.01f;
    me->yaw_fast += rate * 0.01f;
  }
}

// DMP INT 中断服务: 读一个 FIFO 包 → 缓存欧拉角 + 校准后陀螺仪
// 在 EXTI 回调中调用, 一次 I2C 读取同时获得姿态和陀螺仪
void mpu6050_drv_dmp_isr(Mpu6050 *me) {
  if (!me->dmp_enabled) return;

  int16_t gx, gy, gz;
  if (mpu_dmp_read_fifo(&me->pitch, &me->roll, &me->yaw,
                        &gx, &gy, &gz) == 0) {
    me->gx = gx;
    me->gy = gy;
    me->gz = gz;
    me->gyro_z_raw = (float)gz / GYRO_SCALE;
    me->yaw_abs = me->yaw;  // DMP 绝对航向 (四元数, 无漂移)
    me->dmp_data_ready = 1;
  }
}
