# HardC

> **纯 C99 面向对象的电力电子 / 电机控制驱动库** —— 采样、控制、驱动、保护全链路，同一套代码同时跑 STM32 与 C2000。

![Language](https://img.shields.io/badge/language-C99-green)
![Platforms](https://img.shields.io/badge/platform-STM32%20%7C%20C2000-blue)
![license](https://img.shields.io/badge/license-MIT-blue)

HardC 是**组件库**，不是一个工程。它把"控制算法"和"芯片平台"彻底分离：算法是一次写好的库资产，芯片差异被压缩进一层薄薄的 BSP；你要做的只是选拓扑、接外设、写 App 逻辑。配网装配请用 [**yamc**](https://github.com/youyan2000/YamC)。

---

## ✨ 它解决什么问题

传统嵌入式开发，每上一个新项目就要重写一遍采样、控制、保护、驱动。HardC 把这些沉淀成**可复用、跨平台、实时安全**的 C99 库：

| | 不用 HardC | 用 HardC |
|---|---|---|
| 新项目 | 从零抄上次的控制代码 | 选拓扑 → 拉库 → 写 App 逻辑 |
| 换芯片 | 重写外设驱动 | 只动 BSP 一层 |
| 控制算法 | 和硬件耦合在一起 | 算法层与芯片彻底分离 |

核心设计四句话：

1. **纯 C 完成面向对象** —— 继承 + 虚函数表，全 `static inline` 分发，任何 C 工具链（ARM Clang / GCC / TI CGT）都能编译，中断上下文调用安全。
2. **五层架构，职责单向** —— 下层永不反向依赖上层，换平台只动 BSP。
3. **库是资产，拓扑是示例** —— 40+ 组件族是库存，具体拓扑只是从库选配的消费示例。
4. **实时性优先，库级强制** —— 中断分 FAST > SLOW > HMI 三档，`bsp_irq_apply()` 启动时强制校验，配错直接停机。

---

## 🧱 五层架构

```
BSP → Components → Devices → Module → App
```

| 层 | 前缀 | 职责 |
|----|------|------|
| **BSP** | `bsp_*` | 平台抽象：不透明句柄隔离 STM32 ↔ C2000，上层永远不碰寄存器 |
| **Components** | `comp_*` | 纯算法与父类：PID / PWM / PLL / ADC 的"规则"部分，与硬件无关 |
| **Devices** | `<域>_<子类>` | 硬件子类：把父类规则绑定到具体外设与引脚 |
| **Module** | `mod_*` | 业务模块：通过父类句柄驱动，内部不知道硬件细节 |
| **App** | `app_main.*` | 应用入口：根结构体、配置注入、ISR、主循环 |

规则只有一条：**下层不能反向依赖上层**。算法层不知道芯片，业务层不知道寄存器，换平台只动 BSP。

---

## 📦 库内容一览

| 域 | 目录 | 内容 |
|----|------|------|
| **ADC** | `Components/adc` | 直流/交流/Follower 三采样器，DMA 双缓冲，硬件加速抽象 |
| **PID** | `Components/pid` | 14 种子类：Standard/Parallel/DCL/PI/PR/QPR/P2PD/PID/级联…… 统一 `pid_compute(base, t, m)` 入口，输出限幅 + 抗积分饱和内建 |
| **PWM** | `Devices/pwm` | 9 种拓扑：BuckBoost/HalfBridge/FullBridge/Interleaved/Resonant/SEPIC/SPWM/SVPWM/WPT，含移相交错与死区管理 |
| **PLL** | `Components/pll` | 5 种锁相环：SOGI/SRF/Notch/DDSRF/SOGI-FLL |
| **Comm** | `Components/comm` | UART/SPI/I2C/CAN/GPIO 五传输，DMA 事务化，CAN 消息语义，串口协议栈 |
| **Motor** | `Components/motor` | 电机控制域：InstaSPIN 移植、三电阻采样、无感观测器族、编码器 |
| **Codec / Contract** | `Components/codec` / `contract` | CRC/校验和/端序/交织、邮箱/双缓冲/环形队列/闩锁 |
| **Database** | `Components/database` | 闪存键值数据库（主备双块）、CRC/校验和、电量计量（Goertzel 逐谐波） |
| **DSP / Math** | `Components/dsp` / `math` | 硬件加速 FFT/FIR（CMSIS-DSP / C2000Ware / 纯 C 三后端自动选择）、IQMath、向量/复数、数据记录 |
| **Protection** | `Devices/protection` / `Module` | 硬件保护链：过压/过流/过温，三电平逆变器死区时间保护，事件日志 |

每个子系统目录自带 `MANIFEST.yaml` 声明依赖，配网生成器据此自动接线。

---

## 🚀 快速上手

### 方式一：用 yamc 装配到你的工程（推荐）

配一份在售的 CubeMX / CCS 工程，用 [**yamc**](https://github.com/youyan2000/YamC) 一键做"库接入 + 外设探测 + 生成 App + CMake 集成 + 编译"：

```bash
# 1. 拉取本库
git clone https://github.com/youyan2000/HardC.git
# 2. 安装 yamc（GUI / CLI 二合一，见 yamc 仓库 README）
pip install <path/to/YamC>
# 3. 在你的工程上跑一条命令，指定拓扑
yamc cfg_run -d D:/proj/my_psu --topology buck --hardc-path <本库路径> --no-build
```

之后你在 **App 层**写自己的 HMI：按键、OLED、串口命令、CAN 报文——模板里留好了接缝，不碰库内部。

### 方式二：直接用库

把需要的子系统目录（`Components/<域>/` + `Devices/<域>/` + `Module/<域>/`，连同 `MANIFEST.yaml`）拷进工程即可；或用 `yamc scaffold gen` 按拓扑生成。构建时引入 `cmake/HardC.CMake`（提供 `HARDC_DIR` / `HARDC_DRIVER` / 头文件路径），并指定 BSP 平台。

### 写自己的控制组件

继承父类、绑 ops 表、实现算法三个步骤即可加入一个新子类，库的生成器会自动识别并接线。App 层有且只有一组模板文件（`app_main.c/h.tmpl`），禁止自由发挥——所有项目基于同一套入口，方便 yamc 统一注入配置。

---

## 🖥 平台支持

| 平台 | 状态 |
|------|------|
| STM32（F334/G474 已验证） | ✅ ARM Clang / GCC |
| C2000（F280049 已验证） | ✅ TI CGT + SysConfig |

新增平台 = 在 BSP 层补一套句柄实现，验证过的算法层零改动。

---

## 🧪 开发 / 验证

仓库自带 `cmake/` 工具链文件（starm-clang / gcc-arm-none-eabi / c2000-ti-cgt），`.github/workflows/build.yml` 持续构建验证。配网、调参请用 [yamc](https://github.com/youyan2000/YamC)。

---

## 📚 相关

- **[HardC](https://github.com/youyan2000/HardC)** —— 本库（零件库）
- **[YamC](https://github.com/youyan2000/YamC)** —— 装配线：配置 / 接入 / 生成工具链 + GUI / CLI 调参

---

## License

MIT
