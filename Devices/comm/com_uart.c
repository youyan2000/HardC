// UART 字节流传输类实现 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 数据流:
//   RX: HAL UART ISR (HAL_UART_RxCpltCallback) → uart_rx_push → SPSC 环 → MAIN uart_read
//   TX: MAIN uart_write 入环 → HAL_UART_Transmit_IT 逐字节发 → TxCplt → uart_tx_complete 续发
//
// 静态单例注册表 (UART_MAX 条) 把 HAL 弱回调分发到对应实例;
// 重写的 HAL_UART_RxCpltCallback / HAL_UART_TxCpltCallback 是本类对全局弱符号的唯一所有者.
//
// TX 忙等说明: uart_write 先入环再检查 tx_busy, 只有 ISR 消费端会清 tx_busy;
//   uart_tx_complete 采用"先清标志再续发"顺序, 消除"MAIN 恰好入环后被 ISR 清忙"的丢字节窗口
//   (ISR 对 MAIN 是原子抢占, 清忙→再取 期间 MAIN 不可能看到忙=0 而重复启动).

#include "com_uart.h"
#include "container_of.h"

#define UART_MAX 4

// 静态单例注册表: HAL 弱回调按 huart 匹配到实例
static Uart *s_uarts[UART_MAX];

// 注册实例 (幂等)
static void reg(Uart *me) {
  for (int i = 0; i < UART_MAX; i++) {
    if (s_uarts[i] == me) {
      return;
    }
  }
  for (int i = 0; i < UART_MAX; i++) {
    if (s_uarts[i] == NULL) {
      s_uarts[i] = me;
      return;
    }
  }
}

// 注销实例
static void unreg(Uart *me) {
  for (int i = 0; i < UART_MAX; i++) {
    if (s_uarts[i] == me) {
      s_uarts[i] = NULL;
      return;
    }
  }
}

// 从 TX 环取一字节发起单字节中断发送 (环空则不动作)
static void start_tx(Uart *me) {
  uint8_t b;
  if (ring_pop(&me->tx, &b)) {
    me->tx_byte = b;
    me->tx_busy = 1;
    HAL_UART_Transmit_IT(me->huart, &me->tx_byte, 1);
  }
}

// 自检: 句柄已绑定且已初始化
static int self_check_impl(CommBase *base) {
  Uart *me = container_of(base, Uart, base);
  if (me->huart == NULL || base->inited == 0) {
    return -1;
  }
  return 0;
}

// 诊断虚表 — 数据面 (环/收发) 不进虚表
static const CommOps uart_ops = {
    .self_check = self_check_impl,
    .reset = NULL,
};

// -------- 构造 / 析构 / 配置 --------

// 初始化: 契约身份 + 环绑定 + 注册单例 + 启动首段单字节接收
void uart_init(Uart *me, const UartConfig *cfg) {
  comm_base_init(&me->base, "uart");
  me->huart = cfg->huart;
  ring_init(&me->rx, cfg->rx_buf, cfg->rx_size);
  ring_init(&me->tx, cfg->tx_buf, cfg->tx_size);
  me->completion = IO_ASYNC_FLAG;
  me->tx_busy = 0;
  me->rx_byte = 0;
  me->tx_byte = 0;
  me->base.ops = &uart_ops;
  reg(me);
  // 先启动接收再发: 避免先发后收漏掉首个字节
  HAL_UART_Receive_IT(me->huart, &me->rx_byte, 1);
}

// 重配: 换句柄/环缓冲并重启接收 (不改变契约身份)
void uart_set_config(Uart *me, const UartConfig *cfg) {
  me->huart = cfg->huart;
  ring_init(&me->rx, cfg->rx_buf, cfg->rx_size);
  ring_init(&me->tx, cfg->tx_buf, cfg->tx_size);
  me->tx_busy = 0;
  me->rx_byte = 0;
  me->tx_byte = 0;
  reg(me);
  HAL_UART_Receive_IT(me->huart, &me->rx_byte, 1);
}

// 反初始化: 注销单例 + 清状态
void uart_deinit(Uart *me) {
  unreg(me);
  me->base.ops = NULL;
  me->huart = NULL;
  comm_base_deinit(&me->base);
}

// -------- 数据操作 (MAIN 上下文调用) --------

// 写: 入 TX 环, 空闲则启动发送; IO_SYNC 忙等全部发完
ErrorCode uart_write(Uart *me, CommConstData data, IoCompletion comp) {
  uint16_t n = ring_write(&me->tx, data.ptr, data.len);
  if (n < data.len) {
    return ERR_NO_BUFF;
  }
  if (!me->tx_busy) {
    start_tx(me);
  }
  if (comp == IO_SYNC) {
    // 忙等 TX 结束 (仅 CTX_MAIN 调用, IO_SYNC 语义)
    while (me->tx_busy) {
      // 空转: 等 ISR 消费 TX 环
    }
  }
  return ERR_OK;
}

// 读: 从 RX 环读出, 实际读出数写回 data->len (空 → ERR_EMPTY)
ErrorCode uart_read(Uart *me, CommData *data, IoCompletion comp) {
  (void) comp;  // 环读是即时消费, 无异步完成
  uint16_t n = ring_read(&me->rx, data->ptr, data->len);
  if (n == 0u) {
    return ERR_EMPTY;
  }
  data->len = n;
  return ERR_OK;
}

// -------- ISR / 回调入口 --------

// ISR 入口: 入 RX 环, 满则丢弃 (ring_push 返回 false, 不覆盖)
void uart_rx_push(Uart *me, uint8_t byte) {
  (void) ring_push(&me->rx, byte);
}

// TX 完成回调: 先清忙 (释放给 MAIN 重新触发), 环非空则续发下一字节
void uart_tx_complete(Uart *me) {
  me->tx_busy = 0;
  start_tx(me);
}

// HAL 弱回调重写 — RX 完成: 入环并重启接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  for (int i = 0; i < UART_MAX; i++) {
    Uart *me = s_uarts[i];
    if (me != NULL && me->huart == huart) {
      uart_rx_push(me, me->rx_byte);
      HAL_UART_Receive_IT(huart, &me->rx_byte, 1);
      break;
    }
  }
}

// HAL 弱回调重写 — TX 完成: 续发下一字节
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  for (int i = 0; i < UART_MAX; i++) {
    Uart *me = s_uarts[i];
    if (me != NULL && me->huart == huart) {
      uart_tx_complete(me);
      break;
    }
  }
}
