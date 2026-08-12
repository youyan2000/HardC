// 超声波测距驱动 —— CommBase 子类实现
// USART3 发 0xA0 触发 → 收 DataH + DataL + CHK → 16-bit 毫米值
// ultrasonic_tick: 每 10ms 由 TIM3 ISR 调用, 触发测距/超时检测
// ultrasonic_process: USART3 RX 回调中调用, 组装 3 字节回复帧
//
// LitteCar 增强 (2026-07):
// - 校验和验证 CHK = DataH + DataL, 坏帧丢弃并计数
// - 可配置超时 (timeout_ms), 超时时标记 valid=false
// - 诊断计数器暴露: chk_err, ot_cnt

#include "com_ultrasonic.h"
#include "container_of.h"

// -------- ops 实现 --------

// 通过 USART3 阻塞发送指令 (如 0xA0 触发测距)
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  Ultrasonic *me = container_of(base, Ultrasonic, base);
  HAL_UART_Transmit(me->huart, (uint8_t *)dat, len, 100);
}

// 启动 USART3 单字节中断接收
static void bgn_impl(CommBase *base) {
  Ultrasonic *me = container_of(base, Ultrasonic, base);
  HAL_UART_Receive_IT(me->huart, &base->cur, 1);
}

// 读取当前接收到的字节
static uint8_t read_impl(CommBase *base) {
  return base->cur;
}

// 超声波虚函数表
static const CommOps ultrasonic_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 / 析构 --------

// 初始化超声波驱动: 调基类构造 → 绑定 HAL 句柄 → 注册 ops → 接收缓冲清零
void ultrasonic_init(Ultrasonic *me, CommName name, UART_HandleTypeDef *huart) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->huart     = huart;
  me->base.ops  = &ultrasonic_ops;
  me->index     = 0;
  me->dist      = 0;
  me->prev      = 0;
  me->ot_cnt     = 0;
  me->chk_err    = 0;
  me->timeout_ms = 2000;
  me->valid      = false;
  for (int i = 0; i < 4; i++) { me->data[i] = 0; }
}

// 反初始化: 清空 ops 和 HAL 句柄
void ultrasonic_deinit(Ultrasonic *me) {
  me->base.ops = NULL;
  me->huart    = NULL;
}

// -------- 定时调用: 触发新测距 / 超时检测 --------

// 每 10ms 调用: 空闲时发送 0xA0 触发新测距, 超时时标记无效并复位
void ultrasonic_tick(Ultrasonic *me) {
  if (me->index == 0) {
    // 空闲 → 发送测距指令, 启动接收
    me->ot_cnt = 0;
    uint8_t cmd  = 0xa0;
    comm_send(&me->base, &cmd, 1);
    me->index  = 1;
    comm_bgn(&me->base);
  } else {
    // 等待接收中 → 超时后强制复位
    me->ot_cnt++;
    uint16_t threshold = me->timeout_ms / 10;  // tick 周期 10ms
    if (me->ot_cnt >= threshold) {
      me->index  = 0;
      me->ot_cnt = 0;
      me->valid  = false;  // 超时标记无效
    }
  }
}

// -------- 串口回调: 处理超声波模块回复的数据 --------

// USART3 RX 回调中调用: 逐字节接收并组装 3 字节回复帧 (DataH + DataL + CHK)
void ultrasonic_process(Ultrasonic *me) {
  me->data[me->index] = me->base.cur;
  if (me->index >= 3) {
    // 收满 3 字节 → data[1]=DataH, data[2]=DataL, data[3]=CHK
    uint8_t chk = me->data[1] + me->data[2];  // 校验和: DataH + DataL (低 8 位)
    if (chk != me->data[3]) {
      // 校验失败 → 丢弃, 计数, 标记无效
      me->chk_err++;
      me->valid = false;
      me->index = 0;
      return;
    }
    // 校验通过 → 组装距离值
    me->prev = ((uint16_t)me->data[1] << 8) | me->data[2];
    if (me->prev <= 4500) {
      me->dist = me->prev;  // 合理范围才更新有效值 (超声波最大约 4.5m)
      me->valid = true;
    } else {
      me->valid = false;  // 超出范围标记无效
    }
    me->index = 0;  // 回到空闲, 下一 tick 自动触发新测距
  } else {
    me->index++;
    comm_bgn(&me->base);  // 继续收下一字节
  }
}

// -------- LitteCar 增强 API --------

// 返回最新有效距离 (mm), 调用前应先 ultrasonic_is_valid() 确认
uint16_t ultrasonic_get_distance(const Ultrasonic *me) {
  return me->dist;
}

// 最新读数是否通过校验和与量程检查
bool ultrasonic_is_valid(const Ultrasonic *me) {
  return me->valid;
}

// 返回校验和错误累计次数 (可用于诊断链路质量)
uint16_t ultrasonic_get_checksum_errors(const Ultrasonic *me) {
  return me->chk_err;
}

// 设置超时时间 (ms), tick 周期为 10ms 时内部转换为 tick 数
void ultrasonic_set_timeout(Ultrasonic *me, uint16_t timeout_ms) {
  me->timeout_ms = timeout_ms;
}
