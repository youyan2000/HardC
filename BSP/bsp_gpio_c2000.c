// BSP GPIO C2000 后端 — bsp_gpio.h 的 C2000 (F28004x driverlib) 实现
//
// C2000 GPIO 模型: 引脚号全局 (F28004x: GPIO0~59), 无端口概念.
//   BspGpioPin.port     = 占位 (忽略)
//   BspGpioPin.pin      = GPIO 号 (0~59)
//   BspGpioPin.mux_cfg  = 编译期引脚复用值 (pin_map.h 的 GPIO_x_GPIOx, App 注入)
//   BspGpioPin.xint     = 外部中断槽位 XINT1~5 (0 = 不用中断)
//
// 中断模型 (A3 修正, 对照真实 driverlib v1.10.00.00):
//   F28004x 无 GPIO INT1 组 — 边沿中断走 XINT1~5 (gpio.h: GPIO_setInterruptPin
//   第二参数为 GPIO_ExternalIntNum = XINT1..5, 非引脚号). 每个 XINT 是独立 PIE
//   向量 (XINT1/2 在组1, XINT3/4/5 在组12, 见 hw_ints.h), 本后端自注册 ISR 分发.
//   驱动库未封装 XINT 挂起清除 (INTCLR 位) — 与 gpio.h 同样直接写 XINTnCR 寄存器
//   (TRM SPRUIV0 §XINT: bit0=ENABLE, bit1=INTCLR, bit2-3=POLARITY).
//
// 由 cmake/HardC.CMake 的 c2000 分支编译; driverlib.h 由工具链 --preinclude 注入.

#include "bsp_gpio.h"

#include "driverlib.h"

// ======== 电平 / 方向 ========

void bsp_gpio_cfg_output(BspGpioPin *pin) {
  if (pin == NULL) {
    return;
  }
  // 显式解复用为 GPIO 功能 (mux_cfg = pin_map.h 编译期值, 如 GPIO_0_GPIO0)
  GPIO_setPinConfig(pin->mux_cfg);
  GPIO_setDirectionMode(pin->pin, GPIO_DIR_MODE_OUT);
  GPIO_setPadConfig(pin->pin, GPIO_PIN_TYPE_STD);
}

void bsp_gpio_cfg_input(BspGpioPin *pin, BspGpioPull pull) {
  if (pin == NULL) {
    return;
  }
  // 先显式解复用为 GPIO 功能 (若引脚曾被配为 ePWM 等外设 mux, 不设则读回非外部电平)
  GPIO_setPinConfig(pin->mux_cfg);
  GPIO_setDirectionMode(pin->pin, GPIO_DIR_MODE_IN);
  switch (pull) {
  case BSP_GPIO_PULL_UP:
    GPIO_setPadConfig(pin->pin, GPIO_PIN_TYPE_PULLUP);
    break;
  case BSP_GPIO_PULL_DOWN:
    // F28004x 无内部下拉 (gpio.h 的 GPIO_PIN_TYPE 仅 STD/PULLUP/OD/INVERT) —
    // 下拉只能外接电阻, 内部配浮空 (不造假功能)
    GPIO_setPadConfig(pin->pin, GPIO_PIN_TYPE_STD);
    break;
  case BSP_GPIO_PULL_NONE:
  default:
    GPIO_setPadConfig(pin->pin, GPIO_PIN_TYPE_STD);
    break;
  }
  // 输入去抖滤波: 关 (纯电平/边沿, 滤波归上层)
  GPIO_setQualificationMode(pin->pin, GPIO_QUAL_ASYNC);
}

void bsp_gpio_write(BspGpioPin *pin, bool level) {
  if (pin == NULL) {
    return;
  }
  GPIO_writePin(pin->pin, level ? 1u : 0u);
}

bool bsp_gpio_read(BspGpioPin *pin) {
  if (pin == NULL) {
    return false;
  }
  return GPIO_readPin(pin->pin) != 0u;
}

void bsp_gpio_toggle(BspGpioPin *pin) {
  if (pin == NULL) {
    return;
  }
  GPIO_togglePin(pin->pin);
}

// ======== 中断 (XINT1~5) ========

// 回调注册表: 5 个 XINT 槽位, 每槽至多一个引脚/回调
static bsp_gpio_irq_fn s_gpio_cb[5];
static void *s_gpio_ctx[5];
static BspGpioPin s_gpio_pin[5];  // 注册时快照 (回调参数, 含 mux_cfg/xint)

// XINT 槽位 1~5 → GPIO_ExternalIntNum (枚举声明序 0..4 = XINT1..XINT5, 见 gpio.h)
static GPIO_ExternalIntNum xint_enum(uint8_t xint) {
  return (GPIO_ExternalIntNum) (uint16_t) (xint - 1u);
}

// XINT 槽位 → PIE 中断向量
static uint32_t xint_vector(uint8_t xint) {
  switch (xint) {
  case 1u: return INT_XINT1;
  case 2u: return INT_XINT2;
  case 3u: return INT_XINT3;
  case 4u: return INT_XINT4;
  case 5u: return INT_XINT5;
  default: return 0u;
  }
}

// XINT 槽位 → PIE ACK 组 (XINT1/2 在组1, XINT3/4/5 在组12, 见 hw_ints.h)
static uint16_t xint_ack_group(uint8_t xint) {
  return (xint <= 2u) ? INTERRUPT_ACK_GROUP1 : INTERRUPT_ACK_GROUP12;
}

// XINTnCR.INTCLR: 写 1 清 XINTn 挂起标志 (TRM SPRUIV0 §XINT; driverlib v1.10 未封装,
// 与 gpio.h 相同直接写寄存器 — XINT_BASE 见 hw_memmap.h)
#define BSP_GPIO_C2000_XINT_INTCLR 0x0002u

static void xint_clear_flag(uint8_t xint) {
  HWREGH(XINT_BASE + (uint16_t) (xint - 1u)) |= (uint16_t) BSP_GPIO_C2000_XINT_INTCLR;
}

// 边沿 → driverlib 中断类型
static GPIO_IntType gpio_int_type(BspGpioEdge edge) {
  switch (edge) {
  case BSP_GPIO_EDGE_RISE:
    return GPIO_INT_TYPE_RISING_EDGE;
  case BSP_GPIO_EDGE_FALL:
    return GPIO_INT_TYPE_FALLING_EDGE;
  case BSP_GPIO_EDGE_BOTH:
  default:
    return GPIO_INT_TYPE_BOTH_EDGES;
  }
}

// 共享分发: 清挂起 (防残留伪触发) → 查表回调 → 清 PIEACK (否则同组只触发一次)
static void xint_dispatch(uint8_t xint) {
  xint_clear_flag(xint);
  bsp_gpio_irq_fn cb = s_gpio_cb[xint - 1u];
  void *ctx = s_gpio_ctx[xint - 1u];
  if (cb != NULL) {
    cb(&s_gpio_pin[xint - 1u], ctx);
  }
  Interrupt_clearACKGroup(xint_ack_group(xint));
}

// 5 个 XINT ISR — 各自注册到独立 PIE 向量 (Interrupt_register)
#define BSP_GPIO_C2000_XINT_ISR(n)                                     \
  static void bsp_gpio_c2000_xint##n##_isr(void) {                     \
    xint_dispatch(n);                                                  \
  }
BSP_GPIO_C2000_XINT_ISR(1)
BSP_GPIO_C2000_XINT_ISR(2)
BSP_GPIO_C2000_XINT_ISR(3)
BSP_GPIO_C2000_XINT_ISR(4)
BSP_GPIO_C2000_XINT_ISR(5)
#undef BSP_GPIO_C2000_XINT_ISR

// XINT 槽位 → ISR 函数指针 (向量注册表)
static void (*const s_xint_isr[5])(void) = {
    bsp_gpio_c2000_xint1_isr, bsp_gpio_c2000_xint2_isr, bsp_gpio_c2000_xint3_isr,
    bsp_gpio_c2000_xint4_isr, bsp_gpio_c2000_xint5_isr,
};

// PIE 向量注册只做一次 (同一槽位重配时向量不变)
static bool s_vec_registered[5];

int bsp_gpio_enable_irq(BspGpioPin *pin, BspGpioEdge edge, bsp_gpio_irq_fn cb, void *ctx) {
  if (pin == NULL || cb == NULL || pin->pin > 59u || pin->xint < 1u || pin->xint > 5u) {
    return -1;
  }
  uint8_t xint = pin->xint;
  GPIO_ExternalIntNum e = xint_enum(xint);

  // 引脚 → XINTn 输入选择 (XBAR INPUTxSELECT) + 边沿极性 (XINTnCR.POLARITY)
  GPIO_setInterruptPin(pin->pin, e);
  GPIO_setInterruptType(e, gpio_int_type(edge));

  s_gpio_cb[xint - 1u] = cb;
  s_gpio_ctx[xint - 1u] = ctx;
  s_gpio_pin[xint - 1u] = *pin;

  uint32_t vec = xint_vector(xint);
  if (!s_vec_registered[xint - 1u]) {
    Interrupt_register(vec, s_xint_isr[xint - 1u]);
    s_vec_registered[xint - 1u] = true;
  }
  GPIO_enableInterrupt(e);  // XINTnCR.ENABLE
  Interrupt_enable(vec);    // PIE 使能
  return 0;
}

void bsp_gpio_disable_irq(BspGpioPin *pin) {
  if (pin == NULL || pin->xint < 1u || pin->xint > 5u) {
    return;
  }
  uint8_t xint = pin->xint;
  GPIO_ExternalIntNum e = xint_enum(xint);
  GPIO_disableInterrupt(e);
  xint_clear_flag(xint);  // 清残留挂起, 防重新 enable 时触发伪回调
  Interrupt_disable(xint_vector(xint));
  s_gpio_cb[xint - 1u] = NULL;
  s_gpio_ctx[xint - 1u] = NULL;
}
