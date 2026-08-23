// BSP GPIO 硬件抽象接口 —— 平台无关引脚描述 + 物理参数 API (值类型, 零 HAL)
//
// 定位: 与 bsp_pwm.h 同级的 BSP 抽象, 供 Components/Devices/Module 统一操作 GPIO.
// 无去抖、无 active-low 反转 (按键去抖归 Module 层 HMI), 只做电平/中断原语.
//
// 后端清单:
//   STM32: bsp_gpio_stm32.c (HAL GPIO + EXTI, 经 bsp_stm32_hal.h 系列无关)
//   C2000: bsp_gpio_c2000.c (driverlib, pin=0~59 全局引脚号, port 占位;
//                           mux_cfg = pin_map.h 编译期复用值; 中断走 XINT1~5)
//
// 引脚描述为值类型: App 层以 BspGpioPin 注入, 传输类 (如 Spi/Iic) 直接内嵌.

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>
#include <stdbool.h>

// BspGpioPin — 平台无关 GPIO 引脚描述 (值类型, App 层注入)
//   STM32: port = GPIO_TypeDef*, pin = 0~15; mux_cfg/xint = 0 (忽略)
//   C2000: port = 占位(忽略), pin = 0~59 (全局引脚号, F28004x)
//          mux_cfg = 编译期引脚复用值 (pin_map.h 的 GPIO_x_GPIOx, 如 GPIO_0_GPIO0)
//          xint    = 外部中断槽位 XINT1~5 (bsp_gpio_enable_irq 用); 0 = 不使用中断
//
// C2000 App 层构造示例 (A3 接口扩展 — mux_cfg 必填; 用中断时再填 xint):
//   // pin_map.h 经 driverlib.h 注入 (C2000 工程由工具链 --preinclude 提供)
//   BspGpioPin led = { .port = NULL, .pin = 20, .mux_cfg = GPIO_20_GPIO20, .xint = 0 };
//   BspGpioPin key = { .port = NULL, .pin = 24, .mux_cfg = GPIO_24_GPIO24, .xint = 2 };  // XINT2
//   per_led_init(&g_led, "led", led);                       // 输出: 只需 mux_cfg
//   bsp_gpio_enable_irq(&key, BSP_GPIO_EDGE_FALL, cb, ctx); // 中断: 还需 xint 槽位
typedef struct {
  void *port;        // STM32: GPIO_TypeDef*; C2000: 占位 (忽略)
  uint16_t pin;      // STM32: 引脚号 0~15; C2000: 全局引脚号 0~59
  uint32_t mux_cfg;  // C2000: 编译期引脚复用值 (pin_map.h GPIO_x_GPIOx, App 注入); STM32: 0
  uint8_t xint;      // C2000: XINT 槽位 1~5 (仅 bsp_gpio_enable_irq 使用); STM32: 0
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
