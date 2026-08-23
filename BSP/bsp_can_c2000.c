// BSP CAN C2000 后端 — bsp_can.h 的 TMS320F28004x 实现 (driverlib)
//
// 学 libxr STM32CAN 模型 (无 DMA):
//   - 发送: CAN_sendMessage 非阻塞入硬件 mailbox (即发即收).
//   - 接收: CAN_INT 中断 → CAN_readMessage → bsp_can_on_rx → 上层回调.
//
// [待验证] driverlib API 名 (CAN_sendMessage/CAN_readMessage/CAN_enableInterrupt/
//   CAN_INT_OBJ1/INT_CANA0 等) 需在 C2000 工具链核对.
//
// 前提: SysConfig board.c (或工程 setup) 已完成 CAN GPIO/位时序/使能 + 中断注册.
// 本层只做收发 + 回调分发, 不碰 GPIO/位时序配置 — 与 bsp_c2000_adc/epwm 同约定.

#include "bsp_can.h"
#include <stdbool.h>
#include "driverlib.h"

// ======== 模块级状态 (单 CAN-A) ========

static uint32_t s_can_base = 0u;  // CAN 基址 (CANA_BASE)
static BspCanRxFn s_rx_cb = NULL;
static void *s_rx_ctx = NULL;
static BspCanTxDoneFn s_tx_cb = NULL;
static void *s_tx_ctx = NULL;
static bool s_isr_registered;

// ======== CAN 接收中断 — 读帧 → 上层回调 ========
// 与 bsp_c2000_adc_isr 同职责: 只搬移 + 清中断, 不做业务.
__interrupt void bsp_c2000_can_isr(void) {
  // 读尽接收缓冲 (每帧转上层回调)
  while (CAN_getInterruptStatus(s_can_base, CAN_INT_OBJ1) != 0u) {  // [待验证]
    BspCanFrame f = {0};
    uint32_t id = 0u;
    uint32_t ide = 0u;
    uint32_t dlc = 0u;
    CAN_readMessage(s_can_base, CAN_MSG_OBJ1, &id, &ide, &dlc, (uint16_t *) f.data);  // [待验证]
    f.id = id;
    f.ide = (ide != 0u) ? 1u : 0u;
    f.dlc = (dlc > 8u) ? 8u : (uint8_t) dlc;
    if (s_rx_cb != NULL) {
      s_rx_cb((BspCan *) s_can_base, &f, s_rx_ctx);
    }
    CAN_clearInterruptStatus(s_can_base, CAN_INT_OBJ1);  // [待验证]
  }
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);  // CAN-A INT → PIE 组 9 [待验证]
}

// ======== CAN TX 发送完成中断 — 通知上层续发 (M6) ========
// [待验证] CAN_sendMessage 后 TX 完成中断/状态检测 (TRS/TXOK 或 mailbox 空) 需工具链核对
__interrupt void bsp_c2000_can_tx_isr(void) {
  // 发送完成 → 通知上层 (com_can 队列续发 / pending 重试)
  if (s_tx_cb != NULL) {
    s_tx_cb((BspCan *) s_can_base, s_tx_ctx);
  }
  CAN_clearInterruptStatus(s_can_base, CAN_INT_OBJ1);  // [待验证]
  Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);       // [待验证]
}

// ======== BSP 接口 ========

BspCan *bsp_can_bind(void *h) {
  uint32_t base = (uint32_t) h;
  if (base == 0u) {
    return NULL;
  }
  s_can_base = base;
  s_rx_cb = NULL;
  s_rx_ctx = NULL;
  s_tx_cb = NULL;
  s_tx_ctx = NULL;
  return (BspCan *) base;  // 不透明 = CAN 基址
}

void bsp_can_unbind(BspCan *me) {
  (void) me;
  if (s_isr_registered) {
    Interrupt_disable(INT_CANA0);  // [待验证]
  }
  s_can_base = 0u;
  s_rx_cb = NULL;
  s_tx_cb = NULL;
}

void bsp_can_set_rx_cb(BspCan *me, BspCanRxFn cb, void *ctx) {
  (void) me;
  s_rx_cb = cb;
  s_rx_ctx = ctx;
}

void bsp_can_set_tx_done_cb(BspCan *me, BspCanTxDoneFn cb, void *ctx) {
  (void) me;
  s_tx_cb = cb;
  s_tx_ctx = ctx;
}

bool bsp_can_send(BspCan *me, const BspCanFrame *f) {
  (void) me;
  if (f == NULL || f->dlc > 8u) {
    return false;
  }
  // 透传 CAN_sendMessage 返回值: false = mailbox 满/参数错 (com_can pending 重试依赖此契约, M6)
  // [待验证] CAN_sendMessage 返回类型与语义需 C2000 工具链核对
  return CAN_sendMessage(s_can_base, CAN_MSG_OBJ1, f->ide, f->id, f->dlc,
                         (const uint16_t *) f->data) == 0u;  // [待验证]
}

// 平台完成钩子 (STM32 风格; C2000 由 ISR 直接分发, 此处空)
void bsp_can_on_rx(BspCan *me, const BspCanFrame *f) {
  (void) me;
  (void) f;
}

void bsp_can_on_tx_done(BspCan *me) {
  (void) me;
  // C2000 发送完成中断触发时调用 (待实现: CAN 发送完成状态检测)
  if (s_tx_cb != NULL) {
    s_tx_cb((BspCan *) s_can_base, s_tx_ctx);
  }
}
