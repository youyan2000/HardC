// BSP GPIO 硬件抽象接口 —— 平台无关引脚描述 + 物理参数 API (值类型, 零 HAL)
//
// 定位: 与 bsp_pwm.h 同级的 BSP 抽象, 供 Components/Devices/Module 统一操作 GPIO.
// 无去抖、无 active-low 反转 (按键去抖归 Module 层 HMI), 只做电平/中断原语.
//
// 后端清单:
//   STM32: bsp_gpio_stm32.c (HAL GPIO + EXTI, 经 bsp_stm32_hal.h 系列无关)
//   C2000: 本阶段未实现 (BspGpioPin.port 为端口基址, pin=0~31, 保留占位)
//
// 引脚描述为值类型: App 层以 BspGpioPin 注入, 传输类 (如 Spi/Iic) 直接内嵌.

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

// BspGpioPin — 平台无关 GPIO 引脚描述 (值类型, App 层注入)
//   STM32: port = GPIO_TypeDef*, pin = 0~15
//   C2000: port = 端口基址, pin = 0~31 (后端未实现时保留占位)
typedef struct {
  void *port;
  uint16_t pin;
} BspGpioPin;

typedef enum { BSP_GPIO_PULL_NONE, BSP_GPIO_PULL_UP, BSP_GPIO_PULL_DOWN } BspGpioPull;

typedef enum { BSP_GPIO_EDGE_RISE, BSP_GPIO_EDGE_FALL, BSP_GPIO_EDGE_BOTH } BspGpioEdge;

// 中断回调: pin=触发引脚, ctx=注册时传入的上下文
typedef void (*bsp_gpio_irq_fn)(BspGpioPin *pin, void *ctx);

// 配置为推挽输出 (电平由 bsp_gpio_write 决定)
void bsp_gpio_cfg_output(BspGpioPin *pin);

// 配置为输入 (可带上拉/下拉)
void bsp_gpio_cfg_input(BspGpioPin *pin, BspGpioPull pull);

// 写输出电平 (仅输出模式有效)
void bsp_gpio_write(BspGpioPin *pin, bool level);

// 读输入电平 (true=高)
bool bsp_gpio_read(BspGpioPin *pin);

// 翻转输出电平
void bsp_gpio_toggle(BspGpioPin *pin);

// 使能边沿中断并注册回调 (0=成功, 非0=参数错误)
int bsp_gpio_enable_irq(BspGpioPin *pin, BspGpioEdge edge, bsp_gpio_irq_fn cb, void *ctx);

// 关闭边沿中断并注销回调
void bsp_gpio_disable_irq(BspGpioPin *pin);

#endif  // BSP_GPIO_H
