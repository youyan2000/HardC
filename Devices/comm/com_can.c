// CAN 传输类实现 —— CommBase 子类 (Devices/comm, 阶段1 重构)
//
// 发送: HAL_CAN_AddTxMessage, ID 独立参数 (删除旧版 buf[0..3] 编码 StdId 的 hack).
// 接收: 中断里 HAL_CAN_IRQHandler 交给 HAL; MAIN 轮询 can_poll 读 FIFO0,
//   查订阅表按 id 分发回调 — 分发是纯逻辑, host 可测.

#include "com_can.h"
#include "container_of.h"

// HAL 状态 → ErrorCode 映射
static ErrorCode hal_to_ec(HAL_StatusTypeDef st) {
  switch (st) {
  case HAL_OK:
    return ERR_OK;
  case HAL_TIMEOUT:
    return ERR_TIMEOUT;
  case HAL_BUSY:
    return ERR_BUSY;
  default:
    return ERR_FAILED;
  }
}

// 自检: 句柄已绑定且已初始化
static int self_check_impl(CommBase *base) {
  Can *me = container_of(base, Can, base);
  if (me->hcan == NULL || base->inited == 0) {
    return -1;
  }
  return 0;
}

// 诊断虚表 — 数据面 (订阅表) 不进虚表
static const CommOps can_ops = {
    .self_check = self_check_impl,
    .reset = NULL,
};

// -------- 构造 / 析构 / 配置 --------

// 初始化: 契约身份 + 句柄 + 清订阅表 + 启动 CAN 与 FIFO0 通知
void can_init(Can *me, const CanConfig *cfg) {
  comm_base_init(&me->base, "can");
  me->hcan = cfg->hcan;
  me->completion = IO_ASYNC_FLAG;
  me->sub_count = 0;
  for (int i = 0; i < CAN_SUB_MAX; i++) {
    me->subs[i].fn = NULL;
    me->subs[i].ctx = NULL;
  }
  me->base.ops = &can_ops;
  HAL_CAN_Start(me->hcan);
  HAL_CAN_ActivateNotification(me->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  // 通知钩子默认空实现, 帧在 FIFO0 累积, 由 MAIN 的 can_poll 收取分发
}

// 重配: 换句柄并重启 CAN (不改变契约身份)
void can_set_config(Can *me, const CanConfig *cfg) {
  me->hcan = cfg->hcan;
  HAL_CAN_Start(me->hcan);
  HAL_CAN_ActivateNotification(me->hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// 反初始化: 停 CAN + 清状态
void can_deinit(Can *me) {
  HAL_CAN_Stop(me->hcan);
  me->base.ops = NULL;
  me->hcan = NULL;
  me->sub_count = 0;
  comm_base_deinit(&me->base);
}

// -------- 发送 / 订阅 --------

// 发送标准帧: DLC = data.len (0~8), ID 独立参数
ErrorCode can_send(Can *me, uint32_t id, CommConstData data, IoCompletion comp) {
  (void) comp;  // 邮箱添加是即发即收, 无异步完成
  if (data.len > 8u) {
    return ERR_SIZE;
  }
  CAN_TxHeaderTypeDef tx = {
      .StdId = id,
      .ExtId = 0,
      .IDE = CAN_ID_STD,
      .RTR = CAN_RTR_DATA,
      .DLC = data.len,
      .TransmitGlobalTime = DISABLE,
  };
  uint32_t mb;
  return hal_to_ec(HAL_CAN_AddTxMessage(me->hcan, &tx, (uint8_t *) data.ptr, &mb));
}

// 订阅: 同 id 覆盖旧条目, 否则追加, 表满 → ERR_FULL
ErrorCode can_register(Can *me, uint32_t id, can_rx_fn fn, void *ctx) {
  if (fn == NULL) {
    return ERR_ARG;
  }
  for (int i = 0; i < me->sub_count; i++) {
    if (me->subs[i].id == id) {
      me->subs[i].fn = fn;
      me->subs[i].ctx = ctx;
      return ERR_OK;
    }
  }
  if (me->sub_count >= CAN_SUB_MAX) {
    return ERR_FULL;
  }
  me->subs[me->sub_count].id = id;
  me->subs[me->sub_count].fn = fn;
  me->subs[me->sub_count].ctx = ctx;
  me->sub_count++;
  return ERR_OK;
}

// 取消订阅: 按 id 移除, 后移填洞
void can_unregister(Can *me, uint32_t id) {
  for (int i = 0; i < me->sub_count; i++) {
    if (me->subs[i].id == id) {
      for (int j = i; j < me->sub_count - 1; j++) {
        me->subs[j] = me->subs[j + 1];
      }
      me->sub_count--;
      return;
    }
  }
}

// -------- 接收轮询 --------

// 轮询 FIFO0: 收一帧 → 查订阅表分发; 持续收直到 FIFO 空 (GetRxMessage 非 HAL_OK)
void can_poll(Can *me) {
  while (1) {
    CanFrame frame;
    CAN_RxHeaderTypeDef rx;
    if (HAL_CAN_GetRxMessage(me->hcan, CAN_RX_FIFO0, &rx, frame.data) != HAL_OK) {
      break;
    }
    // 扩展帧按 ExtId 匹配 (订阅表按 32-bit id 比较), 标准帧按 StdId
    frame.id = (rx.IDE == CAN_ID_EXT) ? rx.ExtId : rx.StdId;
    // 防御: DLC 超 8 截断 (frame.data 只有 8 字节)
    frame.dlc = rx.DLC > 8u ? 8u : rx.DLC;
    for (int i = 0; i < me->sub_count; i++) {
      if (me->subs[i].id == frame.id && me->subs[i].fn != NULL) {
        me->subs[i].fn(me, &frame, me->subs[i].ctx);
      }
    }
  }
}
