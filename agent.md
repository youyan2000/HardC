# STM32_OOP — 嵌入式 C 面向对象硬件驱动库

本仓库用 ANSI C 实现面向对象模式的 STM32 硬件驱动框架。每个子项目是独立可复用的层级模块。

---

## ⭐ 复用规则（最重要！必须先理解）

> **复用方式有且仅有两种：**
>
> | 方式 | 操作 | 适用场景 |
> |:---|:---|:---|
> | **方式一：直接拷贝** | 复制整个子项目目录 → 改 `board_init.c` 的引脚/定时器/HAL 句柄 → 直接用 | 硬件变了但设备类型不变（如换个引脚、换个 MCU 系列） |
> | **方式二：继承子类** | 写一个新 `.h/.c` → 父类结构体作为第一个成员 → 实现虚函数 → 绑定 ops → 注册到全局句柄 | 需要新类型的设备（如新增 I2C IO 扩展器、新拓扑的 PWM） |
>
> **任何其他"复用"方式都是错的。** 不要修改父类代码来适配子类。不要跨层调用。不要跳过 ops 表直接操作硬件。

---

## 1. 分层架构（最重要：每层只调直接下层，绝不跨层）

| 层 | 目录/文件 | 角色 | 变化时影响 |
|:---|:---|:---|:---|
| **Application** | `app.c` | 应用层 — 启动函数及钩子函数，在初始化和中断中被调用 | 需求变化 |
| **Module** | `module_*` | 业务模块层 — 采样管理、功率控制、通信管理、错误检测，只通过句柄操作 | 业务逻辑变化 |
| **Devices** | `device_*` / `Devices/` | 设备抽象层 — 子类实现 + 板级绑定，通过 Components 层提供的句柄接口分发。隔离主板变化和芯片引脚选择变化 | 主板布线或引脚分配变化 |
| **Components** | `comp_*` / `Components/` | 通用组件层 — 不看寄存器，只看基础功能。PWM 生成、GPIO、通信、滤波、PID 算法等的父类。隔离芯片变化 | MCU 系列变化 |
| **BSP** | `bsp_*` / `BSP/` | 板级支持包 — 适配底层硬件，对 HAL 函数/寄存器操作的轻量封装 | MCU 型号变化 |

**关键规则**：`app_main.c` 是整个项目的唯一 App 入口；Devices 不直接操作寄存器（通过 BSP 或 HAL）。

### 1.1 App 层架构规则 🔥

> **整个项目只有一组 App（`Templates/app_main.c.tmpl` + `app_main.h.tmpl`）。其他都是 Module（`mod_*`）。**
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
conf/*.yaml  →  Python YmaC/yaml_config_builder.py  →  注入 app_main.c 的 /* CONFIG BEGIN/END */ 之间
```
详见 [YmaC/README.md](YmaC/README.md)。

## 2. 公共文件

| 路径 | 用途 |
|------|------|
| `BSP/container_of.h` | Linux 内核经典向下转型宏 — 从基类指针恢复子类指针 |
| `Components/comp_math.h/c` | 数学工具：限幅、绝对值、死区、线性映射、校验和、Quake III 平方根倒数等 |
| `Components/comp_error.h` | 统一错误码 bitmask 系统 (ERROR_SET/CLEAR/IS_SET 宏) |
| `Components/comp_filter.h` | 数字滤波器：一阶低通 (dt 缓存优化) + 二阶巴特沃斯低通 (biquad DFI) |
| `cmake/` | ARM Clang + GCC 工具链文件 (starm-clang.cmake, gcc-arm-none-eabi.cmake) |
| `YmaC/` | YAML → C designated initializer 配置注入工具 (Python GUI/CLI) |
| `conf/` | YAML 配置变体 (default.yaml, aggressive.yaml 等) |

## 3. 项目保障

| 路径 | 用途 |
|------|------|
| [agent.md](agent.md) | 本文件 — AI/人类共读的总纲，完整 OOP 方法论 |
| [LESSONS.md](LESSONS.md) | 调参教训库 (16 条 + 经验模板), git 版本管理, 禁止回退 |

## 4. 子项目总览

| 目录 | 功能 | 基类 | 子类数 |
|------|------|------|--------|
| [ADC-OOP](ADC-OOP/) | ADC 多通道采样 | AdcBase | 3: Follower / DC Sampler / AC Sampler |
| [COM-OOP](COM-OOP/) | 通信外设 | CommBase | 8: UART / SPI / I2C / CAN / Key / MPU6050 / OLED / Ultrasonic |
| [GPO-OOP](GPO-OOP/) | 通用输出 | GpoBase | 5: LED / Laser / Beep / Buzzer / Fan |
| [PID-OOP](PID-OOP/) | PID 控制器 | PidBase | 4: Standard / P2PD / PR / QPR + Cascade |
| [PWM-OOP](PWM-OOP/) | 电力电子 PWM | PwmBase | 5: BuckBoost / HalfBridge / FullBridge / Interleaved / Resonant |

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
led_base.h        — LedBase 结构体, LedOps typedef, 分发函数声明
led_base.c        — 分发函数实现 (led_on, led_off 等)
led_gpio.c        — LedGpio 结构体, gpio_ops, led_gpio_init
led_pwm.c         — LedPwm 结构体, pwm_ops, led_pwm_init
led_i2c.c         — LedI2c 结构体, i2c_ops, led_i2c_init
leds.h            — extern 句柄声明 (g_led_error, g_led_status, ...)
board_init.c      — 实例化对象, 绑定句柄
app.c             — #include "leds.h", 使用句柄
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

### 方式一：直接拷贝子项目 — 硬件变了，设备类型不变

改 `board_init.c` 即可，其他地方一行不动：

1. 复制整个子项目目录到目标工程
2. 确保 `BSP/container_of.h` 和 `Components/comp_math.h/c` 加入 include path
3. 修改 `board_init.c`：改引脚、定时器、HAL 句柄以适配你的硬件
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

## 8. 各子项目详细文档

每个子项目的 `agent.md` 包含该子项目的继承树、文件清单、依赖、和具体复用示例：

- [ADC-OOP/agent.md](ADC-OOP/agent.md) — ADC 多通道采样
- [COM-OOP/agent.md](COM-OOP/agent.md) — 通信外设
- [GPO-OOP/agent.md](GPO-OOP/agent.md) — 通用输出
- [PID-OOP/agent.md](PID-OOP/agent.md) — PID 控制器
- [PWM-OOP/agent.md](PWM-OOP/agent.md) — 电力电子 PWM
