# Components —— 算法与契约层

HardC 五层架构 L2。这一层只做**不依赖芯片的事**：定义接口契约（父类 + ops 虚表）和平台无关的纯算法。它不知道 STM32 和 C2000 的区别，也不知道引脚和寄存器——那些听 BSP 的和 Devices 的。

## 这层干什么

- **接口契约**：每个域的父类 + ops 虚函数表（`PidBase`/`PidOps`、`PwmBase`、`AdcBase`、`CommBase`、`PllBase`、`PowerStage`…）。子类在 Devices 层继承这些父类、绑定 ops。
- **纯算法组件**：与硬件无关的算法，如 `comp_math`（IQMath/向量/复数）、`comp_codec`（CRC/校验和/RS/Viterbi）、`comp_dlog` / `comp_pfc` / `comp_protection` / `comp_database` / `comp_power_metrology`（Goertzel 逐谐波电量计量）等。
- **统一平台无关入口**：如 `pid_compute(base, target, measure)` 在基类里做好了输出限幅和抗积分饱和调度，子类只实现"算法本尊"。

## 按域划分（14 个，各自含 MANIFEST.yaml 声明依赖）

| 域 | 内容 |
|----|------|
| **adc** | 采样/校准/触发抽象父类 |
| **codec** | CRC / 校验和 / 字节序 / Viterbi / 交织 / RS / ASK |
| **comm** | 通信父类：CommBase + DMA 事务模型（comp_dma_rx/tx） |
| **contract** | 跨上下文原语：comp_io、双缓冲 / 闩锁 / 环形 / 邮箱 |
| **database** | 闪存键值数据库（主备双块、顺序写入） |
| **dsp** | 频谱与信号处理：FFT / 滤波器 / 谐波分析 |
| **math** | IQMath 包装、error/complex、硬件加速数学宏 |
| **motor** | 电机控制算法父类 + InstaSPIN + 无感观测器族 |
| **peripheral** | 外设抽象父类：output / sensor / MPU |
| **pid** | 统一 PID 基类 PidBase + 14 种子类的"规则"部分 |
| **pll** | 锁相环父类 PllBase（PD→LF→VCO 结构） |
| **power** | 功率级/三相/整流/常见拓扑父类 |
| **protection** | 保护链：过压/过流/过温、三电平死区保护 |
| **pwm** | 脉冲调制父类 + 正弦波生成 |

## 边界（别把什么放这里）

- ❌ 禁止 include HAL / Driverlib / 任何平台头 —— 硬件走 `BSP/` 不透明句柄
- ❌ 不放具体芯片实现（那是 Devices 层）
- ❌ 不放业务状态机（那是 Module 层）

## 这个层怎么保证质量

纯算法组件保持 `-Wall -Wextra -Werror` 零警告、可在 host 上做单元测试（不依赖目标板）。

---

> 层级总览与五层职责、跨上下文规则（FAST/SLOW/HMI 三档）见 `../README.md`。
