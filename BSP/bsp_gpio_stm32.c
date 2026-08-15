// BSP GPIO STM32 后端 —— bsp_gpio.h 的 HAL 实现 (GPIO 电平 + EXTI 中断)
//
// 定位: STM32 后端, 经 bsp_stm32_hal.h 系列无关头选具体 HAL 系列.
// C2000 后端本阶段未实现 (bsp_gpio.h 头注释已列后端清单).
//
// EXTI 统一分发: 重写 HAL 弱回调 HAL_GPIO_EXTI_Callback, 按触发引脚查注册表
//   分发到 bsp_gpio_enable_irq 注册的回调. 外部工程如需其他 EXTI 用法,
//   必须走 bsp_gpio_enable_irq 注册, 不能直接重写本回调 (符号冲突).

#include "bsp_gpio.h"
#include "bsp_stm32_hal.h"

// EXTI 注册表: 16 条引脚槽, 与 s_exti_pin 平行 (回调参数需要引脚值)
static struct {
  bsp_gpio_irq_fn cb;
  void *ctx;
} s_exti[16];

// 与 s_exti 平行的引脚值 (使能 IRQ 时快照, 回调时传给用户)
static BspGpioPin s_exti_pin[16];

// EXTI 引脚号 → NVIC IRQn 映射 (0~4 独立线, 5~9 / 10~15 分组线)
static IRQn_Type exti_irqn(uint16_t pin) {
  if (pin <= 4u) {
    return (IRQn_Type) (EXTI0_IRQn + pin);
  }
  if (pin <= 9u) {
    return EXTI9_5_IRQn;
  }
  return EXTI15_10_IRQn;
}

// 除 except_pin 外, 该 IRQ 组线是否还有其他已注册引脚
// (EXTI5~9 共线 EXTI9_5_IRQn, EXTI10~15 共线 EXTI15_10_IRQn; 0~4 各占独立线)
static bool exti_group_has_other(uint16_t except_pin, IRQn_Type irqn) {
  for (int i = 0; i < 16; i++) {
    if (i == except_pin) {
      continue;
    }
    if (s_exti[i].cb != NULL && exti_irqn((uint16_t) i) == irqn) {
      return true;
    }
  }
  return false;
}

// 配置为推挽输出: 无上拉, 高速 (F1 需 Speed, 其余系列忽略该字段)
void bsp_gpio_cfg_output(BspGpioPin *pin) {
  GPIO_InitTypeDef g;
  g.Pin = pin->pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init((GPIO_TypeDef *) pin->port, &g);
}

// 配置为输入: Pull 按 BspGpioPull 映射
void bsp_gpio_cfg_input(BspGpioPin *pin, BspGpioPull pull) {
  GPIO_InitTypeDef g;
  g.Pin = pin->pin;
  g.Mode = GPIO_MODE_INPUT;
  switch (pull) {
  case BSP_GPIO_PULL_UP:
    g.Pull = GPIO_PULLUP;
    break;
  case BSP_GPIO_PULL_DOWN:
    g.Pull = GPIO_PULLDOWN;
    break;
  case BSP_GPIO_PULL_NONE:
  default:
    g.Pull = GPIO_NOPULL;
    break;
  }
  HAL_GPIO_Init((GPIO_TypeDef *) pin->port, &g);
}

// 写输出电平
void bsp_gpio_write(BspGpioPin *pin, bool level) {
  HAL_GPIO_WritePin((GPIO_TypeDef *) pin->port, pin->pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 读输入电平
bool bsp_gpio_read(BspGpioPin *pin) {
  return HAL_GPIO_ReadPin((GPIO_TypeDef *) pin->port, pin->pin) == GPIO_PIN_SET;
}

// 翻转输出电平
void bsp_gpio_toggle(BspGpioPin *pin) {
  HAL_GPIO_TogglePin((GPIO_TypeDef *) pin->port, pin->pin);
}

// 使能边沿中断: HAL_GPIO_Init 配 EXTI 模式 (HAL 内部处理 AFIO/SYSCFG 映射),
// 登记回调并打开 NVIC
int bsp_gpio_enable_irq(BspGpioPin *pin, BspGpioEdge edge, bsp_gpio_irq_fn cb, void *ctx) {
  if (pin == NULL || cb == NULL || pin->pin > 15u) {
    return -1;
  }
  GPIO_InitTypeDef g;
  g.Pin = pin->pin;
  switch (edge) {
  case BSP_GPIO_EDGE_RISE:
    g.Mode = GPIO_MODE_IT_RISING;
    break;
  case BSP_GPIO_EDGE_FALL:
    g.Mode = GPIO_MODE_IT_FALLING;
    break;
  case BSP_GPIO_EDGE_BOTH:
  default:
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    break;
  }
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init((GPIO_TypeDef *) pin->port, &g);

  uint16_t i = pin->pin;
  s_exti[i].cb = cb;
  s_exti[i].ctx = ctx;
  s_exti_pin[i] = *pin;

  IRQn_Type irqn = exti_irqn(i);
  HAL_NVIC_SetPriority(irqn, 0, 0);
  HAL_NVIC_EnableIRQ(irqn);
  return 0;
}

// 关闭边沿中断: 清注册表; 仅当同组线无剩余注册引脚时才关 NVIC
void bsp_gpio_disable_irq(BspGpioPin *pin) {
  if (pin == NULL || pin->pin > 15u) {
    return;
  }
  uint16_t i = pin->pin;
  IRQn_Type irqn = exti_irqn(i);
  s_exti[i].cb = NULL;
  s_exti[i].ctx = NULL;
  // 共线组 (EXTI9_5 / EXTI15_10) 还有别的注册引脚 → 只清表项, 不关整条 NVIC
  if (!exti_group_has_other(i, irqn)) {
    HAL_NVIC_DisableIRQ(irqn);
  }
}

// HAL 弱回调重写 — EXTI 统一分发: GPIO_Pin 可能含多个置位引脚,
// 逐个查注册表分发. 外部 EXTI 使用必须经 bsp_gpio_enable_irq 注册.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  for (int i = 0; i < 16; i++) {
    if ((GPIO_Pin & (1u << i)) == 0u) {
      continue;
    }
    if (s_exti[i].cb != NULL) {
      s_exti[i].cb(&s_exti_pin[i], s_exti[i].ctx);
    }
  }
}
