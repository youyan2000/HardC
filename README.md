# HardC

**纯 C99 面向对象的电力电子 / 电机控制硬件驱动库** —— 覆盖采样、控制、驱动、保护全链路，一套代码同时跑 STM32 与 C2000，适配任意拓扑。

HardC 是一个**组件库**，不是一个工程。它把"控制算法"和"芯片平台"彻底分离：算法是一次写好的库资产，芯片差异被压缩进一层薄薄的 BSP；你要做的只是选拓扑、接外设、写 App 逻辑。

---

## 设计理念

### 1. 纯 C 完成面向对象

不用 C++、不用宏魔法，用 C99 自带的能力实现继承和多态：

- **继承**：子类结构体以父类为第一成员，`container_of` 下溯取子类指针
- **多态**：每个组件家族一张 ops 虚函数表（如 `PidOps`），由构造函数绑定，基类只管理物理契约，不假设任何状态
- 代价为零：全部 `static inline` 分发，无虚表间接调用开销

对嵌入式的意义：**任何 C 工具链都能编译**（ARM Clang / GCC / TI CGT），没有 C++ runtime 依赖，中断上下文里调用也安全。

### 2. 五层架构，职责单向

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

### 3. 库是资产，拓扑是示例

库中已沉淀 40+ 组件族（PID 14 个子类、PWM 9 种拓扑、PLL 5 种锁相环、通信 5 种传输、保护、数据库……），它们是**库存**，不是被某个工程捆绑的死代码。具体拓扑（buck、超级电容三相交错……）只是从库里选配的消费示例。你要新拓扑时，配 YAML 选组件即可，库本体不用裁剪。

### 4. 实时性优先，库级强制

中断分成严格三档：**FAST（控制）> SLOW（监控）> HMI（通信/按键）**。采样 → 控制 → 发波 → 快保护在最高优先级一条链路走完；`bsp_irq_apply()` 启动时强制校验优先级配置，配错了直接停机，不允许带病运行。

---

## 库内容一览

| 域 | 内容 |
|----|------|
| **ADC** | 直流/交流/Follower 三采样器，DMA 双缓冲，硬件加速抽象 |
| **PID** | 14 种子类：Standard/Parallel/DCL/PI/PR/QPR/P2PD/PID/级联…… 统一 `pid_compute(base, t, m)` 入口，输出限幅 + 抗积分饱和内建 |
| **PWM** | 9 种拓扑：BuckBoost/HalfBridge/FullBridge/Interleaved/Resonant/SEPIC/SPWM/SVPWM/WPT，含移相交错与死区管理 |
| **PLL** | 5 种锁相环：SOGI/SRF/Notch/DDSRF/SOGI-FLL |
| **Comm** | UART/SPI/I2C/CAN/GPIO 五传输，DMA 事务化，CAN 消息语义，串口协议栈 |
| **Motor** | 电机控制域：InstaSPIN 移植、三电阻采样、无感观测器族、编码器 |
| **Protection** | 硬件保护链：过压/过流/过温，三电平逆变器死区时间保护，事件日志 |
| **存储** | 闪存键值数据库（主备双块）、CRC/校验和、电量计量（Goertzel 逐谐波） |
| **DSP/Math** | 硬件加速 FFT/FIR（CMSIS-DSP / C2000Ware / 纯 C 三后端自动选择）、IQMath、向量/复数 |

每个子系统目录自带 `MANIFEST.yaml` 声明依赖，生成器据此自动接线。

---

## 快速上手

### 方式一：在你的 CubeMX / CCS 工程里用（推荐）

配套工具链 **yamc** 一键完成"库接入 + 外设探测 + 编译集成"：

1. 拉取仓库：`git clone https://github.com/youyan2000/HardC.git`
2. 安装 yamc：见 [yamc](https://github.com/youyan2000/YamC)（YAML 调参 / GUI 配置 / CLI 二合一）
3. 在你的工程上跑一条命令，指定拓扑，yamc 自动：做外设表 → 生成 App 骨架 → 注入 CMake → 编译出 `.elf/.hex`

之后你在 **App 层**写自己的 HMI：按键、OLED、串口命令、CAN 报文——模板里留好了接缝，不碰库内部。

### 方式二：直接用库

把需要的子系统目录（`Components/<域>/` + `Devices/<域>/` + `Module/<域>/`，连同 `MANIFEST.yaml`）拷进工程即可；或用 `scaffold.py gen` 按拓扑生成。构建时引入 `cmake/HardC.CMake`（提供 `HARDC_DIR` / `HARDC_DRIVER` / 头文件路径），并指定 BSP 平台。

### 写自己的控制组件

继承父类、绑 ops 表、实现算法三个步骤即可加入一个新子类，库的生成器会自动识别并接线。App 层有且只有一组模板文件（`app_main.c/h.tmpl`），禁止自由发挥——所有项目基于同一套入口，方便 yamc 统一注入配置。

---

## 平台支持

| 平台 | 状态 |
|------|------|
| STM32（F334/G474 已验证） | ✅ ARM Clang / GCC |
| C2000（F280049 已验证） | ✅ TI CGT + SysConfig |

新增平台 = 在 BSP 层补一套句柄实现，验证过的算法层零改动。

---

## 相关仓库

- **[HardC](https://github.com/youyan2000/HardC)** —— 本库（零件库）
- **[YamC](https://github.com/youyan2000/YamC)** —— 装配线：配置/接入/生成工具链