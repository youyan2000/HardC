// USART 通信驱动 —— CommBase 子类实现
// 封装 HAL UART 操作, 提供 send(阻塞) / bgn(中断接收) / read(读当前字节)
// 发送时暂时关闭 USART1 NVIC 中断, 防止 ISR 和 TX 抢 HAL 状态机

#include "com_uart.h"
#include "container_of.h"

// -------- ops 实现 --------

// 阻塞发送: 关闭 UART NVIC 中断, 防止 ISR 和 TX 抢 HAL 状态机
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  Uart *me = container_of(base, Uart, base);
  HAL_NVIC_DisableIRQ(USART1_IRQn);
  HAL_UART_Transmit(me->huart, (uint8_t *)dat, len, 100);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

// 启动单字节中断接收，收到后 HAL_UART_RxCpltCallback 触发
static void bgn_impl(CommBase *base) {
  Uart *me = container_of(base, Uart, base);
  HAL_UART_Receive_IT(me->huart, &base->cur, 1);
}

// 读取当前接收到的最后一个字节
static uint8_t read_impl(CommBase *base) {
  return base->cur;
}

// USART 虚函数表
static const CommOps uart_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 / 析构 --------

// 初始化 UART 驱动: 调基类构造 → 绑定 HAL 句柄和接收缓冲区 → 注册 ops
void uart_init(Uart *me, CommName name, UART_HandleTypeDef *huart, uint8_t *rxb) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->huart     = huart;
  me->base.ops  = &uart_ops;
  me->base.buf  = rxb;
  me->base.cur  = 0;
}

// 反初始化: 清空 ops 和 HAL 句柄
void uart_deinit(Uart *me) {
  me->base.ops = NULL;
  me->huart    = NULL;
}
