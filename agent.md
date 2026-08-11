# C-OOP — 嵌入式 C 面向对象硬件驱动库

本仓库用 ANSI C 实现面向对象模式的嵌入式硬件驱动框架。按 bsp-dev-c 风格组织为五层扁平目录（BSP → Components → Devices → Module → App），文件前缀区分子系统域。跨平台：STM32 (HAL/HRTIM) + TI C2000 (ePWM/CLA) + 纯C回退。

---

## ⭐ 复用规则（最重要！必须先理解）

> **复用方式有且仅有两种：**
>
> | 方式 | 操作 | 适用场景 |
> |:---|:---|:---|
> | **方式一：直接拷贝** | 复制需要的 Component + Device + Module 文件 → 改引脚/定时器/HAL 句柄 → 直接用 | 硬件变了但设备类型不变（如换个引脚、换个 MCU 系列） |
> | **方式二：继承子类** | 写一个新 `.h/.c` → 父类结构体作为第一个成员 → 实现虚函数 → 绑定 ops → 注册到全局句柄 | 需要新类型的设备（如新增 I2C IO 扩展器、新拓扑的 PWM） |
>
> **任何其他"复用"方式都是错的。** 不要修改父类代码来适配子类。不要跨层调用。不要跳过 ops 表直接操作硬件。

---

## 1. 分层架构（最重要：每层只调直接下层，绝不跨层）

| 层 | 目录 | 命名前缀 | 角色 | 变化时影响 |
|:---|:---|:---|:---|:---|
| **Application** | `App/` | `app_main.*` | 应用入口 — 根结构体、ISR 钩子、BackgroundTask、board_init | 需求变化 |
| **Module** | `Module/` | `mod_*` | 业务模块 — 采样管理、功率控制、通信管理、错误检测，只通过 Base* 句柄操作 | 业务逻辑变化 |
| **Devices** | `Devices/` | `<域>_<子类>` | 设备抽象 — 子类实现 + 板级绑定，通过 Components 层句柄分发 | 主板布线或引脚分配变化 |
| **Components** | `Components/` | `comp_*` | 通用组件 — 不看寄存器，只看基础功能。PWM/PID/ADC/通信等父类 + ops 虚表 | MCU 系列变化 |
| **BSP** | `BSP/` | `bsp_*` | 板级支持包 — 不透明句柄，隔离 MCU 厂商差异 (STM32 ↔ TI C2000) | MCU 型号变化 |

**关键规则**：`app_main.c` 是整个项目的唯一 App 入口；Devices 不直接操作寄存器（通过 BSP 或 HAL）。

### 1.1 App 层架构规则 🔥

> **整个项目只有一组 App（`App/app_main.c.tmpl` + `App/app_main.h.tmpl`）。其他都是 Module（`mod_*`）。**
> 参考: WEILAI_SuperCap (`User/app/app_main.c`) + LitteCar CMake (`User/Application/app_main.c`)。

| 规则 | 说明 |
|:---|:---|
| **根结构体值包含** | `ProjectRoot` 嵌入所有 Device + Module 实例，零 malloc |
| **指针注入** | Module 之间通过 `Base*` 指针引用，`board_init()` 一次性解析，此后不变 |
| **配置与运行时分离** | Config 是纯数据 POD（YAML 注入目标），Instance 持有运行时状态 + ops 虚表 |
| **ISR 只做快操作** | 控制计算、状态机 tick、寄存器操作。禁止 printf、软件 I2C、OLED |
| **BackgroundTask 做慢操作** | 串口应答、传感器打印、OLED 刷新、软件 I2C 读取 |
| **ISR 调用顺序** | 传感器采样 → HMI/命令分发 → 控制算法 → 执行器输出（PWM 最后） |
| **全局访问** | `extern ProjectRoot g_root` 让 Module 可直读兄弟模块状态 |

**YmaC 配置注入流程：**
```
Config/params/*.yaml  →  Python YmaC/yaml_config_builder.py  →  注入 app_main.c 的 /* CONFIG BEGIN/END */ 之间
```
详见 [YmaC/README.md](YmaC/README.md)。

## 2. 公共文件

| 路径 | 用途 |
|------|------|
| **BSP** | |
| `BSP/container_of.h` | Linux 内核经典向下转型宏 — 从基类指针恢复子类指针 |
| `BSP/bsp_dsp.h` | **硬件加速抽象层** — sqrt/biquad, 平台检测 (CMSIS-DSP/C2000Ware/纯C回退) |
| `BSP/bsp_pwm.h` | **PWM BSP 接口** — 不透明句柄 + 物理参数 API (duty/Hz/ns/deg) |
| `BSP/bsp_adc.h` | **ADC BSP 接口** — 校准/启动抽象 (bsp_adc_calibrate / bsp_adc_start_dma) |
| `BSP/bsp_delay.h/c` | 微秒延时抽象 |
| **Components** | (前缀 = 父类域) |
| `Components/comp_math.h/c` | 数学工具：限幅、绝对值、死区、线性映射、校验和、硬件加速 sqrt |
| `Components/comp_error.h` | 统一错误码 bitmask 系统 (ERROR_SET/CLEAR/IS_SET 宏) |
| `Components/comp_filter.h` | 数字滤波器：一阶低通 + 二阶巴特沃斯低通 (biquad DFI) |
| `Components/comp_protection.h` | 保护框架：阈值检测、去抖、分级响应 |
| `Components/comp_adc.h/c` | ADC 父类：AdcBase + AdcOps 虚表 |
| `Components/comp_pwm.h/c` | PWM 父类：PwmBase + PwmOps 虚表 |
| `Components/comp_pid.h/c` | PID 父类：PidBase + PidOps 虚表 |
| `Components/comp_gpo.h/c` | 通用输出父类：GpoBase + GpoOps 虚表 |
| `Components/comp_comm.h/c` | 通信父类：CommBase + CommOps 虚表 |
| `Components/comp_motor.h/c` | 电机父类：MotorBase + MotorOps 虚表 |
| `Components/comp_mpu.h` / `comp_mpu_dmp.c` | MPU6050 DMP 算法层 |
| **App** | |
| `App/app_main.c.tmpl` | App 实现模板 — board_init、ISR、BackgroundTask |
| `App/app_main.h.tmpl` | App 头模板 — 根结构体、配置 POD |
| **工程** | |
| `Config/params/` | YAML 配置变体 (default.yaml, aggressive.yaml 等) |
| `Config/topologies/` | 拓扑级 YAML 配置 (将来: six_switch_acdc.yaml 等) |
| `YmaC/` | YAML → C designated initializer 配置注入工具 (Python GUI/CLI) |
| `cmake/` | ARM Clang + GCC + C2000 工具链文件 |

## 3. 项目保障

| 路径 | 用途 |
|------|------|
| [agent.md](agent.md) | 本文件 — AI/人类共读的总纲，完整 OOP 方法论 |
| [LESSONS.md](LESSONS.md) | 调参教训库 (17 条 + 经验模板), git 版本管理, 禁止回退 |

## 3.5 BSP 硬件加速抽象层 🔌

> **Components 层禁止直接 include 平台加速库 (`arm_math.h`, `C2000Ware_dsp.h` 等)。所有硬件加速走 BSP 抽象。**

| BSP 文件 | 抽象内容 | M4F/M7 (HW) | M0+/M3 (SW) | C2000 | 纯C回退 |
|----------|---------|-------------|-------------|-------|--------|
| `BSP/bsp_dsp.h` | sqrt, biquad IIR | CMSIS-DSP FPU SIMD | CMSIS-DSP 软件库 | C2000Ware TMU/CLA | 牛顿迭代 + DFI |
| `BSP/bsp_pwm.h` | PWM 不透明句柄 + 物理参数API | bsp_hrtim.c (HRTIM) | — | bsp_c2000_epwm.c (ePWM) | — |
| `BSP/bsp_adc.h` | ADC 校准 + DMA 启动 | bsp_adc_stm32.c | — | bsp_adc_c2000.c | — |

**不透明句柄模式：**
```c
// BSP 头文件 — 平台无关
typedef void BspPwmHandle;           // 上层只看到不透明指针

// 物理参数 API (推荐): BSP 内部换算为寄存器值
void bsp_pwm_set_duty_f(BspPwmHandle *h, BspPwmTimer t, float duty);    // [0.0, 1.0]
void bsp_pwm_set_freq_hz(BspPwmHandle *h, BspPwmTimer t, uint32_t hz);
void bsp_pwm_set_deadtime_ns(BspPwmHandle *h, BspPwmTimer t, uint32_t ns);

// 寄存器级 API (保留): 供需要精细控制的拓扑使用
void bsp_update_duty(BspPwmHandle *h, BspPwmTimer t, uint32_t cmp1, uint32_t cmp3);
```

**平台自动检测 (`BSP/bsp_dsp.h`) — 四级能力分级：**
```c
// BSP_DSP_ARCH: 2=硬件加速 1=软件库 3=C2000Ware 0=纯C回退
#if defined(__ARM_FEATURE_DSP) && __FPU_PRESENT  // M4F/M7 → CMSIS-DSP HW
  #define BSP_DSP_ARCH 2
#elif defined(__ARM_ARCH_6M__) || __ARM_ARCH_7M__ // M0/M0+/M3 → CMSIS-DSP SW
  #if __has_include("arm_math.h")
    #define BSP_DSP_ARCH 1
  #else
    #define BSP_DSP_ARCH 0
  #endif
#elif defined(__TMS320C2000__)                   // C2000 → TMU/CLA
  #define BSP_DSP_ARCH 3
#else
  #define BSP_DSP_ARCH 0                          // 纯C回退
#endif
```

**关键原则：** 每个 `#if` 分支的行为语义等价（滤波就是滤波，不是直通）。回退路径不是"后续实现"的占位符——它就在当前版本真实工作。见 [LESSONS.md](LESSONS.md) #34.

## 4. 子系统总览（按文件前缀区分）

| 域 | Component | Devices 子类 | Module | 句柄头文件 |
|----|-----------|-------------|--------|-----------|
| **ADC** | `comp_adc.h/c` | `adc_follower`, `adc_dc_sampler`, `adc_ac_sampler` | `mod_sampler` | `adcs.h` |
| **COM** | `comp_comm.h/c` | `com_uart`, `com_spi`, `com_i2c`, `com_can`, `com_key`, `com_mpu6050`, `com_oled`, `com_ultrasonic` | `mod_comm`, `mod_cmd_dispatch`, `mod_serial_proto` | `comms.h` |
| **GPO** | `comp_gpo.h/c` | `gpo_led`, `gpo_laser`, `gpo_beep`, `gpo_buzzer`, `gpo_fan` | — | `gpos.h` |
| **PID** | `comp_pid.h/c` | `pid_standard`, `pid_cascade`, `pid_p2pd`, `pid_parallel`, `pid_pr`, `pid_qpr` | — | `pids.h` |
| **PWM** | `comp_pwm.h/c` | `pwm_buckboost`, `pwm_half_bridge`, `pwm_full_bridge`, `pwm_interleaved`, `pwm_resonant` | `mod_powerctrl` | `pwms.h` |
| **Motor** | `comp_motor.h/c` | `motor_tim` | — | — |

> 所有文件扁平化存放在 `Components/`、`Devices/`、`Module/` 目录。文件前缀即为域标识，不需要子目录嵌套。

## 4. OOP 核心模式（C 语言实现）

### 4.1 封装（Encapsulation）

- `.h` 只放公共 API 声明
- `.c` 中 `static` 函数/变量为私有；常量用 `static const`
- 函数命名：`<类名>_<方法>`（如 `led_on`、`adc_read_ch`）
- `_init` / `_deinit` 成对放在 API 顶部和底部
- 数据进结构体；结构体指针 `me` 进每个函数作为第一个参数

### 4.2 继承（Inheritance）— 结构体嵌入

**父类结构体作为子类结构体的第一个成员**。这保证 `&derived.base == &derived`（向上转型的关键）。

```c
// 父类
typedef struct { const char *name; int state; } LedBase;

// 子类 — base 必须是第一个成员
typedef struct {
    LedBase base;      // ← 第一个成员 = 继承
    int32_t pin;
    bool    on_level;
} LedGpio;

// 子类构造器调用父类构造器
void led_gpio_init(LedGpio *me, const char *name, int32_t pin, bool on_level) {
    led_base_init(&me->base, name);  // 调父类构造器
    me->pin = pin;
    me->on_level = on_level;
}
```

### 4.3 虚函数表（Virtual Function Table / ops table）— 三步构建

**Step 1:** typedef 函数指针签名
```c
typedef void (*led_on_fn)(LedBase *me);
typedef void (*led_off_fn)(LedBase *me);
```

**Step 2:** 构建 ops 表结构体
```c
typedef struct {
    led_on_fn  on;    // [必须] 打开
    led_off_fn off;   // [必须] 关闭
    led_set_brightness_fn set_brightness;  // [可选] NULL = 不支持
} LedOps;
```

**Step 3:** 在父类结构体中添加 ops 指针
```c
struct LedBase {
    const char   *name;
    int           state;
    const LedOps *ops;  // ← 虚表指针（vptr）
};
```

### 4.4 多态（Polymorphism）— 通过 ops 分发

**父类分发函数**（应用层看到的唯一 API）：
```c
// 必须操作 — assert 确保子类实现
void led_on(LedBase *me) {
    assert(me->ops->on);
    me->ops->on(me);
}

// 可选操作 — NULL 检查后静默跳过
void led_set_brightness(LedBase *me, int val) {
    if (me->ops->set_brightness) {
        me->ops->set_brightness(me, val);
    }
}
```

**子类实现 + ops 注册**：
```c
// led_gpio.c
typedef struct { LedBase base; uint32_t pin; uint8_t on_level; } LedGpio;

static void gpio_on(LedBase *base) {
    LedGpio *me = container_of(base, LedGpio, base);  // 向下转型
    hal_gpio_write(me->pin, me->on_level);
}

static const LedOps gpio_ops = {
    .on  = gpio_on,
    .off = gpio_off,
    // .set_brightness = NULL  ← GPIO 不支持调光
};

void led_gpio_init(LedGpio *me, const char *name, uint32_t pin, uint8_t on_level) {
    led_base_init(&me->base, name);
    me->pin = pin;
    me->on_level = on_level;
    me->base.ops = &gpio_ops;  // ← 构造时绑定 ops
}
```

### 4.5 向上转型（Upcasting）— 类型擦除到父类指针

因为 `base` 是第一个成员，`&gpio_led.base` 和 `&gpio_led` 地址相同。将具体子类擦除为父类指针存储：

```c
// leds.h — 应用层只看到句柄
extern LedBase *g_led_error;
extern LedBase *g_led_status;

// board_init.c — 板级绑定具体类型到句柄
static LedGpio gpio_err;
static LedPwm  pwm_status;

void board_init(void) {
    led_gpio_init(&gpio_err, "error", 10, LED_ACTIVE_HIGH);
    led_pwm_init(&pwm_status, "status", 1, 50);

    g_led_error  = &gpio_err.base;    // 向上转型: LedGpio* → LedBase*
    g_led_status = &pwm_status.base;  // 向上转型: LedPwm*  → LedBase*
}
```

### 4.6 向下转型（Downcasting）— container_of

在子类的虚函数实现内部，从父类指针恢复子类指针：

```c
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

// 用法：
static void gpio_on(LedBase *base) {
    LedGpio *me = container_of(base, LedGpio, base);  // 父→子
    hal_gpio_write(me->pin, me->on_level);
}
```

`container_of` 无论 `base` 是否第一个成员都能正确计算偏移。但放在第一位保证向上转型的地址相等。

### 4.7 Initcall（自注册）— 可选

自动模块初始化，不需要修改 `main.c`。利用链接器 section：

```c
#define __initcall(fn) \
    __attribute__((used, section(".initcall"))) \
    static init_fn __initcall_##fn = fn

// 在每个模块：
static void my_module_init(void) { /* ... */ }
__initcall(my_module_init);

// main.c 遍历 section 调用所有 init 函数
```

## 5. 完整框架模板

创建新的驱动族（如 LED、Motor、Sensor）时，按以下文件布局：

```
Components/comp_xxx.h/c  — XxxBase 结构体, XxxOps typedef, 分发函数
Devices/xxx_variant.h/c   — 具体子类 × N (xxx_gpio, xxx_pwm, xxx_i2c...)
Devices/xxxs.h            — extern 全局句柄声明
Module/mod_xxx.h/c        — (可选) 业务模块，组合多个 Device
App/app_main.c            — #include "xxxs.h", 使用句柄
```

## 6. 代码风格约定

| 规则 | 示例 |
|------|------|
| 函数命名 | `<module>_<verb>` — `led_on`, `adc_read_ch`, `gpo_set` |
| 结构体命名 | `PascalCase` — `LedBase`, `GpoLed`, `AdcOps` |
| 变量命名 | `snake_case` — `on_level`, `raw_cap`, `num_ch` |
| 指针参数 | 统一用 `me`（非 `this` / `self`），始终第一个参数 |
| 构造/析构 | 始终 `_init` / `_deinit` 成对 |
| 虚表命名 | `static const`，在 `.c` 中，名为 `<module>_ops` |
| 全局句柄 | `g_<name>` 前缀（如 `g_led_error`, `g_buck`） |
| 必须/可选 ops | 必须操作用 `assert`；可选操作用 `if (ops->xxx)` NULL 检查 |
| Include guard | 按层统一格式，见下方 §6.1 |
| 文件组织 | 一个子类一个 `.c` 文件；父类 + ops typedef 在一个 `.h` |

### 6.1 Include Guard 命名规范

按文件所在层统一格式，**禁止裸 `XXX_H`**：

| 层 | 格式 | 示例 |
|:---|:---|:---|
| **Components** (`comp_*.h`) | `COMP_<NAME>_H` | `COMP_PID_H`, `COMP_MOTOR_H`, `COMP_PROTECTION_H` |
| **Devices** (子项目 `*.h`) | `<SUB>_<NAME>_H` | `PWM_BUCKBOOST_H`, `ADC_FOLLOWER_H`, `COM_UART_H` |
| **Module** (`mod_*.h`) | `MOD_<NAME>_H` | `MOD_POWERCTRL_H`, `MOD_CMD_DISPATCH_H` |
| **BSP** (`bsp_*.h`) | `BSP_<NAME>_H` | `BSP_DELAY_H`, `BSP_MPU6050_H`, `BSP_PWM_H` |
| **全局句柄** (`*s.h`) | `<NAME>S_H` | `PWMS_H`, `ADCS_H`, `PIDS_H`, `COMMS_H`, `GPOS_H` |
| **其他** (根级) | `<NAME>_H` | `CONTAINER_OF_H`, `LESSONS_H` |

**规则：**
- `<NAME>` = 文件名（不含 `.h`），全部大写，用下划线分隔
- 新增文件必须遵守；修改现有文件时顺手修正
- 不要用裸 `XXX_H`（如 `ADCS_H` 应写全而不是简写）

## 7. 复用方式（详见顶部 ⭐复用规则）

### 方式一：直接拷贝需要文件 — 硬件变了，设备类型不变

改引脚/定时器/HAL 句柄即可，其他地方一行不动：

1. 复制需要的 `Component + Device + Module` 文件到目标工程
2. 确保 `BSP/container_of.h` 和 `Components/comp_math.h/c` 加入 include path
3. 修改 Device 文件中的硬件句柄以适配你的硬件
4. 应用层通过全局句柄头文件（`gpos.h` / `pwms.h` / `comms.h` / `pids.h`）操作，不感知子类

### 方式二：基于父类继承新子类 — 需要新类型的设备

**绝对不能改父类代码。** 只新增文件，三步走：

1. **定义结构体**（`.h`）：
   - 父类结构体作为第一个成员
   - 添加子类特有的硬件资源（GPIO、TIM、I2C 地址等）
   - 声明 `_init` / `_deinit`

2. **实现虚函数**（`.c`）：
   - 每个虚函数 `static`，用 `container_of(base, SubType, base)` 恢复子类指针
   - 组装 `static const XxxOps my_ops = { .op1=..., .op2=..., .optional=NULL };`
   - 构造器中调父类 `_init` → 覆盖基类字段 → `me->base.ops = &my_ops`
   - 析构器中关闭硬件 → 调父类 `_deinit`

3. **注册到系统**：
   - 在全局句柄头文件中添加 `extern` 声明
   - 在 `board_init.c` 中实例化 + 绑定句柄

**黄金法则**：
- 父类必须是结构体第一个成员（保证向上转型地址相等）
- 有状态（积分器等）→ 在构造器中初始化；子类各自独立
- 所有硬件资源放子类，绝不污染父类
- 虚表中"必须"操作用 assert，"可选"操作用 NULL + if 检查

## 8. 子系统参考文档

各子系统的继承树、文件清单、依赖、和具体复用示例见下方。每个子系统对应一组文件前缀（全部在 `Components/` + `Devices/` + `Module/` 目录下）。

### 8.1 ADC 子系统 — 多通道采样

**继承树：**
```
AdcBase (虚表 + 名称 + DMA 缓冲区指针 + 位置偏差)
├── AdcFollower   — 8 路红外循迹传感器 (二值化 + 独热码 + 位置映射)
├── AdcDcSampler  — 通用直流采样器 (N 通道 EMA 滤波 + k·raw+b 线性校准)
└── AdcAcSampler  — 三相交流采样器 (差分采样 + 三相重构 + RMS + Vdc)
```

**AdcOps 虚表（3 必须 + 2 可选）：** start_dma(必须) / read_ch(必须) / process(必须) / get_sum2(可选) / get_ch_bin(可选)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h/c`, STM32 HAL

### 8.2 COM 子系统 — 通信外设

**继承树：**
```
CommBase (虚表 + 名称 + 接收缓冲区 + 当前字节)
├── ComUart      — USART 驱动 (阻塞 TX / 中断逐字节 RX)
├── ComSpi       — SPI 主模式 + CS 引脚控制
├── ComI2c       — I2C 主模式 + 设备地址
├── ComCan       — CAN 消息帧收发 + 硬件过滤器
├── ComKey       — 按键驱动 (GPIO 读取 + 双击/长按状态机)
├── ComMpu6050   — MPU6050 六轴传感器 (DMP + 回退双模式)
├── ComOled      — OLED SSD1306 薄包装
└── ComUltrasonic — 超声波测距 (触发→接收→解码)
```

**CommOps 虚表（4 必须 + 2 可选）：** send(必须) / bgn(必须) / read(必须) / avail(必须) / is_ok(可选) / reset(可选)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h/c`, `Components/comp_mpu.h`, STM32F1 HAL

### 8.3 GPO 子系统 — 通用输出

**继承树：**
```
GpoBase (虚表 + 名称)
├── GpoLed    — LED (双模: GPIO 开关 / PWM 调光)
├── GpoLaser  — 激光笔 (GPIO 开关, 安全优先: 默认关闭)
├── GpoBeep   — 有源蜂鸣器 (GPIO 开关)
├── GpoBuzzer — 无源蜂鸣器 (PWM 调音)
└── GpoFan    — 风扇 (PWM 调速)
```

**GpoOps 虚表（2 必须 + 1 可选）：** on(必须) / off(必须) / set_brightness(可选)

**关键教训：** 子类按设备类型分（LED/Laser/Buzzer），不按电气机制分（GPIO/PWM）。安全关键外设（激光）必须在 `_init()` 中显式写 OFF。见 LESSONS.md #28, #29。

### 8.4 PID 子系统 — 控制器

**继承树：**
```
PidBase (虚表 + dt + out_min/out_max + anti_windup)
├── PidStandard  — 标准位置式 PID + 变速积分 + 微分先行 + 钳位抗饱和
├── PidCascade   — 级联 PID (外环 + 内环, 组合模式非继承)
├── PidP2PD      — 点到点微分 PID (循迹专用)
├── PidParallel  — 并联 PID (独立 P/I/D 通道)
├── PidPR        — 比例谐振 (PR) 控制器
└── PidQPR       — 准比例谐振 (QPR) 控制器
```

**PidOps 虚表（2 必须 + 1 可选）：** compute(必须) / reset(必须) / on_saturation(可选)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h/c`, `<math.h>`

### 8.5 PWM 子系统 — 电力电子拓扑

**继承树：**
```
PwmBase (虚表 + 模式 + 通道数 + 频率 + 占空比限幅 + 运行状态)
├── PwmBuckBoost   — 单路 Buck/Boost (可选同步整流)
├── PwmHalfBridge  — 半桥互补 PWM (中心对齐 + 死区)
├── PwmFullBridge  — 全桥移相 PWM (A/B 两腿 + 移相角控制功率)
├── PwmInterleaved — 多相交错并联 PWM (N 相均匀错相 360°/N)
└── PwmResonant    — 谐振变频 PWM (50% 固定占空比 + 变频控制)
```

**PwmOps 虚表（4 必须 + 2 可选）：** start(必须) / stop(必须) / set_duty(必须) / set_freq(必须) / is_running(可选) / get_status(可选)

**BSP 物理参数 API (推荐)：** `bsp_pwm_config_ch`, `bsp_pwm_set_duty_f`, `bsp_pwm_set_freq_hz`, `bsp_pwm_set_deadtime_ns`, `bsp_pwm_set_phase_deg`, `bsp_pwm_set_complementary`, `bsp_pwm_isr`

**依赖：** `BSP/container_of.h`, `BSP/bsp_pwm.h`, `BSP/bsp_dsp.h`, `<math.h>`

### 8.6 Motor 子系统 — 直流电机

**继承树：**
```
MotorBase (虚表 + 编码器 + PID 句柄)
└── MotorTim — TIM PWM + AB 相编码器
```

**MotorOps 虚表：** setspeed(必须) / readspeed(必须) / readposition(必须) / stop(必须)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h`, STM32F1 HAL
