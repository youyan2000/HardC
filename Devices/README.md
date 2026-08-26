# Devices —— 具体子类实现层

HardC 五层架构 L3。这一层继承 Components 的父类，把它们**落成能跑的硬件/算法子类**：绑到具体外设、具体引脚、或某个具体算法变体。一个子类一个文件。

## 这层干什么

- **继承 + 多态的落点**：把 Components 的父类作为结构体第一成员，`container_of` 下溯取子类指针，构造器里绑定 ops 虚表——OOP 的"继承与多态"在这里真正发生。
- **按域分文件**：`<域>_<子类>`（如 `adc_dc_sampler` / `pwm_svpwm` / `pid_standard`），按域归子目录，各带 MANIFEST.yaml。

## 子类目录一览（7 个域）

| 域 | 内容 |
|----|------|
| **adc** | DC / AC / Follower 三采样器（DMA 双缓冲、去 HAL、复用 comp_adc_sig） |
| **comm** | UART / SPI / I2C / CAN / GPIO 传输子类 + 串口协议栈 |
| **pid** | 14 种子类的硬件变体（Standard / Parallel / DCL / PI / PR / QPR / P2PD / 级联…） |
| **pll** | SOGI / SRF / Notch / DDSRF / SOGI-FLL 五子类 |
| **pwm** | BuckBoost / HalfBridge / FullBridge / Interleaved / SEPIC / SPWM / SVPWM / WPT 等 |
| **motor** | 编码器 / TIM 采样 / 无感观测器落点 |
| **peripheral** | 输出（LED/蜂鸣器/风扇）、传感器（OLED/MPU6050…）等 18 子类 |

## 边界（别把什么放这里）

- ❌ 依赖 BSP 接口，**禁止绕过 BSP 直接滚裸寄存器** —— 硬件差异归 BSP
- ❌ 不做业务逻辑 / 状态机（那是 Module 层的责任）
- ❌ 不做平台无关的纯算法（那属于 Components）

## 怎么加一个新子类

继承父类（父类作第一成员）→ 实现 ops 表对应函数 → 在构造器绑定 ops → MANIFEST 声明依赖。yamc / scaffold 会自动识别并接线。

---

> 层级总览见 `../README.md`；父类契约精确定义在各域 Components 头文件。
