// CAN 总线通信驱动 —— CommBase 子类实现
// 封装 HAL CAN 操作, 提供消息帧级别的发送/接收
//
// CommOps 语义适配:
//   send: dat[0..3]=StdId(小端), dat[4..len-1]=payload → HAL_CAN_AddTxMessage
//   bgn:  HAL_CAN_Start + 激活 FIFO0 消息挂起通知
//   read: 轮询 FIFO0, 有新消息时缓存并返回 DLC, 无则返回 0
//
// 扩展 API: can_send_msg / can_poll / can_set_filter

#include "com_can.h"
#include "container_of.h"

// -------- ops 实现 --------

// 发送: dat[0..3]=32位 StdId(小端), dat[4..]=payload
// 例: dat = {0x05, 0x02, 0x00, 0x00, 0xAA, 0xBB} → ID=0x205, DLC=2, data=0xAA,0xBB
static void send_impl(CommBase *base, const uint8_t *dat, uint16_t len) {
  Can *me = container_of(base, Can, base);

  if (len < 5) return;  // 至少需要 4 字节 ID + 1 字节数据

  uint32_t id  = (uint32_t)dat[0] | ((uint32_t)dat[1] << 8)
               | ((uint32_t)dat[2] << 16) | ((uint32_t)dat[3] << 24);
  uint8_t  dlc = (uint8_t)(len - 4);
  if (dlc > 8) dlc = 8;

  CAN_TxHeaderTypeDef tx_header = {
    .StdId = id,
    .ExtId = 0,
    .IDE   = CAN_ID_STD,
    .RTR   = CAN_RTR_DATA,
    .DLC   = dlc,
    .TransmitGlobalTime = DISABLE,
  };
  uint32_t tx_mbox;
  HAL_CAN_AddTxMessage(me->hcan, &tx_header, (uint8_t *)(dat + 4), &tx_mbox);
}

// 启动 CAN: 开启 CAN 外设 + 激活 FIFO0 消息挂起中断
static void bgn_impl(CommBase *base) {
  Can *me = container_of(base, Can, base);
  HAL_CAN_Start(me->hcan);
  HAL_CAN_ActivateNotification(me->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// 读取: 轮询 FIFO0, 有新消息时缓存并返回 DLC, 无则返回 0
static uint8_t read_impl(CommBase *base) {
  Can *me = container_of(base, Can, base);
  return can_poll(me);
}

// CAN 虚函数表
static const CommOps can_ops = {
  .send = send_impl,
  .bgn  = bgn_impl,
  .read = read_impl,
};

// -------- 构造 / 析构 --------

// 初始化 CAN 驱动: 调基类构造 → 绑定 HAL 句柄 → 注册 ops → 接收缓存清零
void can_init(Can *me, CommName name, CAN_HandleTypeDef *hcan) {
  comm_base_init(&me->base);
  me->base.name = name;
  me->hcan      = hcan;
  me->base.ops  = &can_ops;
  me->rx_id     = 0;
  me->rx_dlc    = 0;
  for (int i = 0; i < 8; i++) { me->rx_data[i] = 0; }
}

// 反初始化: 停止 CAN → 清空 ops 和 HAL 句柄
void can_deinit(Can *me) {
  HAL_CAN_Stop(me->hcan);
  me->base.ops = NULL;
  me->hcan     = NULL;
}

// -------- 扩展 API: CAN 消息帧级别操作 --------

// 发送标准帧 (11-bit ID, 0~8 字节数据, DLC 自动 = len)
void can_send_msg(Can *me, uint32_t id, const uint8_t *dat, uint8_t dlc) {
  CAN_TxHeaderTypeDef tx_header = {
    .StdId = id,
    .ExtId = 0,
    .IDE   = CAN_ID_STD,
    .RTR   = CAN_RTR_DATA,
    .DLC   = dlc,
    .TransmitGlobalTime = DISABLE,
  };
  uint32_t tx_mbox;
  HAL_CAN_AddTxMessage(me->hcan, &tx_header, (uint8_t *)dat, &tx_mbox);
}

// 轮询 RX FIFO0: 有新消息时填充 rx_id/rx_dlc/rx_data, 返回 dlc; 无消息返回 0
uint8_t can_poll(Can *me) {
  if (HAL_CAN_GetRxFifoFillLevel(me->hcan, CAN_RX_FIFO0) == 0) {
    return 0;
  }
  CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(me->hcan, CAN_RX_FIFO0, &rx_header,
                           me->rx_data) != HAL_OK) {
    return 0;
  }
  me->rx_id  = rx_header.StdId;
  me->rx_dlc = rx_header.DLC;
  return me->rx_dlc;
}

// 配置硬件接收过滤器: 单 filter 掩码模式, FIFO0
void can_set_filter(Can *me, uint32_t filter_id, uint32_t mask_id) {
  CAN_FilterTypeDef filter = {
    .FilterIdHigh       = (uint16_t)(filter_id << 5),
    .FilterIdLow        = 0,
    .FilterMaskIdHigh   = (uint16_t)(mask_id << 5),
    .FilterMaskIdLow    = 0,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterBank         = 0,
    .FilterMode         = CAN_FILTERMODE_IDMASK,
    .FilterScale        = CAN_FILTERSCALE_32BIT,
    .FilterActivation   = ENABLE,
    .SlaveStartFilterBank = 0,
  };
  HAL_CAN_ConfigFilter(me->hcan, &filter);
}
