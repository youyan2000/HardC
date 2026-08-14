# HardC — 嵌入式 C 面向对象硬件驱动库

本仓库用 ANSI C 实现面向对象模式的嵌入式硬件驱动框架。按 bsp-dev-c 风格组织为五层扁平目录（BSP → Components → Devices → Module → App），文件前缀区分子系统域。跨平台：STM32 (HAL/HRTIM) + TI C2000 (ePWM/CLA) + 纯C回退。

---

## ⚙️ AI 工作准则（行为契约）

> 本文件同时是给 AI 助手的**行为手册**。以下规则是硬约束，与 CLAUDE.md 的共同约定互补：CLAUDE.md 面向人与 AI 双方，本节只约束 AI 的执行方式。

1. **计划先行** — 任何非平凡改动（新功能 / 重构 / 重组 / 涉及 3+ 文件）必须先给出 plan 并获批准，再动手实现。禁止大范围"边做边改"的自由发挥。
2. **每动作更新 HISTORY + LESSONS** — 每完成一个动作/阶段，同步更新 [docs/debug/history/](docs/debug/history/README.md)（阶段、commit 归因、错误与修正）与 [docs/debug/LESSONS.md](docs/debug/LESSONS.md)（新教训按经验模板，禁止回退）。文档落后于代码 = 违约。
3. **遵守代码风格** — 一律遵循 §6 代码风格约定（2 空格缩进、K&R 大括号、命名规范、include guard），提交前用 `.clang-format` / `.clang-tidy` 校验。
4. **Git 纪律** — 不自动提交；用户允许后**及时**按子系统拆分 commit（见 CLAUDE.md 提交规范）；提交前 `git status` 核对范围。
5. **写-审双 Agent** — 任何代码生成走强制 `code-review-workflow`（Writer → Reviewer），Reviewer 不可跳过（纯 .md / YAML 修改例外，commit 注明 `no-review: <原因>`）。

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
>
> **HardC 是库不是可执行工程：** 未调用的组件/函数是**库存**（供未来拓扑按需选用），不是死代码。审查时**禁止**因"无调用者"删除或标注废弃库代码。详见 [LESSONS.md](docs/debug/LESSONS.md) #56。

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
> 参考: `User/app/app_main.c` + CMake `User/Application/app_main.c`。

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
**YmaC 拓扑选择器（GUI Tab2/Tab3）：** 扫 `Config/topologies/<topo>.yaml` 拓扑目录（buck 等，`status: ready` 才可生成）→ 选拓扑 → 合成 `Config/projects/<name>.yaml` 并调 scaffold 生成 `build/gen/<name>/` → 参数表（schema 来自拓扑 `params:`，slot 与 0xFB 帧同源）→ 离线注入物化 `app_main.c` + 运行时串口下发（Tab3, pyserial）。
详见 [YmaC/README.md](YmaC/README.md)。

### 1.2 三上下文执行契约 🔥

> 项目的运行时骨架是**三件事**：硬实时闭环、监控、人机交互。三件事时序要求不同，跑在三个上下文。

| 上下文 | 承载 | 优先级 | 要求 |
|--------|------|--------|------|
| **CTX_FAST** | 硬实时闭环（采样→控制→发波→快保护） | 高 | 周期确定、执行时间 ≪ 周期、不可差错 |
| **CTX_SLOW** | 监控（慢保护去抖、状态聚合、心跳、喂狗） | 中 | 软实时、可抖动、确认问题即关断 |
| **CTX_MAIN** | 人机交互（HMI、协议、日志、调参） | 低 | 非实时、跟得上人的速度即可 |

**全序 + 单一抢占源 → 调用树即上下文。** CTX_FAST 由控制定时器 ISR 承载，CTX_SLOW 由独立监控定时器 ISR 承载（或 FAST 降频驱动），CTX_MAIN 是主循环。三者优先级总序，只有 FAST 能抢占。因此不需要线程调度、锁、`in_isr` 判断——一个函数跑在哪个上下文，看它从哪个 App 钩子被调，编译期即定。

**App 钩子 ↔ 上下文映射（固定命名，xrobot 侧生成代码按此接线）：**

| App 钩子 | 上下文 | 何时跑 |
|----------|--------|--------|
| `App_OnControlTick()` | CTX_FAST | 控制定时器 ISR（如 10kHz） |
| `App_OnSlowTick()` | CTX_SLOW | 监控定时器 ISR（如 1kHz） |
| `BackgroundTask()` | CTX_MAIN | 主循环 |

**每上下文能做什么：**
- **CTX_FAST**：只做确定的事——采样 fetch/process、控制计算、发波、快保护、交接（写 Latest / 入 SPSC / 写邮箱 / 置 Flag）。禁止 printf、软件 I2C、OLED、malloc、一切阻塞。
- **CTX_SLOW**：慢保护去抖、状态聚合、心跳/喂狗、软关断判决。可轻微抖动；确认故障即置关断，硬件封波负责及时。监控三件套 = [bsp_watchdog.h](BSP/bsp_watchdog.h)（硬件喂狗）+ comp_protection.h `Heartbeat`/`Debounce`（死锁判定/去抖分级）——见 §2。
- **CTX_MAIN**：所有耗时 I/O——printf、OLED、软件 I2C、协议解析、调参响应、从 SPSC 出队日志。

**跨上下文数据只用五原语交接，禁止裸共享变量：** PingPong 双缓冲（DMA→FAST，[Components/contract/comp_double_buffer.h](Components/contract/comp_double_buffer.h)）/ Latest-value 锁存（FAST→SLOW/MAIN，[Components/contract/comp_latch.h](Components/contract/comp_latch.h)）/ SPSC 环形缓冲（ISR→MAIN，[Components/contract/comp_ring.h](Components/contract/comp_ring.h)）/ Command 邮箱（MAIN→FAST，周期边界生效，[Components/contract/comp_mailbox.h](Components/contract/comp_mailbox.h)）/ Event-Flag（ISR→MAIN）。五者均已物化：前四者为 static inline 头文件；**Event-Flag = [comp_protection.h](Components/protection/comp_protection.h) §5 `DeferredAction`**——ISR 置位 / MAIN 轮询清零（LibXR Event 的刻意简化：无阻塞等待，因 HardC 无线程，"等待"= 各上下文按自身周期轮询），错误分级归 [comp_error.h](Components/math/comp_error.h) bitmask。五原语全部落地；**PingPong→ADC 已接线**：`adc_dc_sampler` 内嵌 `DoubleBuffer`，DMA 完成 ISR（`adc_dc_sampler_on_dma_complete`，经 `bsp_adc_restart_dma` 重装）标 pending，FAST 每周期 `adc_dc_sampler_fetch` 切快照 + 只碰活动块（fetch/process 分离，撕裂读消除；安全前提=ADC 由控制定时器触发，每控制周期一次完成）。**首个 Module 消费者：** ModBuck（[Module/power/mod_buck.h](Module/power/mod_buck.h)）——Latest 锁存写 vout 遥测 / Command 邮箱 `mod_buck_set_vref` 周期边界生效 / SPSC 环推保护事件（fire-and-forget，`IO_NONE` 语义）。

**五原语源自 LibXR（bsp-dev-c），非自创：** 参照 `LibXR/src/structure/queue/spsc_queue.*`（SPSCQueue）、`src/structure/double_buffer.*`（DoubleBuffer）、`src/middleware/event.*`（Event）、`src/middleware/message/topic.*`（Topic + Sync/ASync/Queued/Callback 订阅者）。本仓库翻译为纯C static inline（无模板 → 字节/单槽/值语义），各头文件已注明来源；与 LibXR 的刻意差异（如 mailbox 单槽覆盖 vs QueuedSubscriber 排队）在头文件里显式声明。参照 LibXR 公开仓库（QDU-Robomaster/bsp-dev-c）学习，各头文件标"来源: LibXR"处为本仓库移植。

**I/O 完成契约：** 发起 I/O 时定完成行为——`IO_ASYNC_CB` / `IO_ASYNC_FLAG` / `IO_SYNC`（仅 MAIN）/ `IO_NONE`（显式忽略），枚举见 [Components/contract/comp_io.h](Components/contract/comp_io.h)。完成行为写进接口，不许悄悄丢掉。

**归属约定：** 每个 Module 在 MANIFEST 声明 `ctx: fast|slow|main`；tick 函数按上下文命名（`_tick_fast` / `_tick_slow`）或在注释声明上下文；App 层按上下文接线，Module 不决定自己跑在哪。

## 2. 公共文件

| 路径 | 用途 |
|------|------|
| **BSP** | |
| `BSP/container_of.h` | Linux 内核经典向下转型宏 — 从基类指针恢复子类指针 |
| `BSP/bsp_dsp.h` | **硬件加速抽象层** — sqrt/biquad, 平台检测 (CMSIS-DSP/C2000Ware/纯C回退) |
| `BSP/bsp_dsp_fir.h` | **FIR 滤波器加速抽象** — float FIR (BspFirInst, 4后端: CMSIS-DSP/C2000/Q15定点/纯C) + Q15 独立接口 (BspFirQ15) |
| `BSP/bsp_dsp_fft.h` | **FFT 加速抽象** — RFFT + CFFT (BspRfftInst/BspCfftInst), 3 后端分发 |
| `BSP/bsp_pwm.h` | **PWM BSP 接口** — 不透明句柄 + 物理参数 API (duty/Hz/ns/deg) |
| `BSP/bsp_adc.h` | **ADC BSP 接口** — 校准/启动抽象 (bsp_adc_calibrate / bsp_adc_start_dma) |
| `BSP/bsp_watchdog.h` | **看门狗 BSP 接口** — IWDG/C2000 WDT 抽象 (bsp_watchdog_init/feed), 移植自 LibXR Watchdog/STM32Watchdog |
| `BSP/bsp_watchdog_stm32.c` | 看门狗 STM32 后端 — HAL IWDG, 分频/重载自动计算 (LibXR stm32_watchdog.cpp) |
| `BSP/bsp_watchdog_c2000.c` | 看门狗 C2000 后端 — WDCR/WDKEY 喂狗 (Phase 3 按数据手册核对) |
| `BSP/bsp_delay.h/c` | 微秒延时抽象 |
| **Components** | (前缀 = 父类域) |
| `Components/math/comp_math.h` | **数学工具 — 全库唯一 π/2π float 常量源 (M_PI/M_2PI)** + 硬件加速宏 (MATH_SQRT/MATH_ISQRT/MATH_ABS, 工程可覆盖) + 限幅/绝对值/死区/线性映射 |
| `Components/comp_error.h` | 统一错误码 bitmask 系统 (ERROR_SET/CLEAR/IS_SET 宏) |
| `Components/comp_filter.h` | 数字滤波器：一阶低通 + 二阶巴特沃斯低通 (biquad DFI) |
| `Components/comp_iir.h` | **IIR 数字补偿器库** — DF22/DF23/2P2Z float + Q15/Q31 定点, 控制环补偿 |
| `Components/comp_sgen.h` | **信号发生器库** — 7 种发生器 (定频/扫频/HP1单音/HP2双音/T3D三音/Profile分段/死区线) |
| `Components/comp_fft_window.h` | **FFT 窗函数库** — 18 种窗 float+Q31, 相干增益/ENBW, 与 bsp_dsp_fft.h 配合 |
| `Components/protection/comp_protection.h` | 保护框架：阈值检测、去抖、分级响应 |
| `Components/comp_adc.h/c` | ADC 父类：AdcBase + AdcOps 虚表 |
| `Components/comp_pwm.h/c` | PWM 父类：PwmBase + PwmOps 虚表 |
| `Components/comp_pid.h/c` | PID 父类：PidBase + PidOps 虚表 |
| `Components/comp_pi_reg4.h` | **4 状态 PI 调节器** — 设定值滤波 + P + I + 前馈, 双抗积分饱和 |
| `Components/comp_gpo.h/c` | 通用输出父类：GpoBase + GpoOps 虚表 |
| `Components/comp_comm.h/c` | 通信父类：CommBase + CommOps 虚表 |
| `Components/comp_motor.h/c` | 电机父类：MotorBase + MotorOps 虚表 |
| `Components/comp_bldc_instaspin.h` | **InstaSPIN-BLDC 无感方波** — 3 阶段启动 (ALIGN/OPENLOOP/CLOSED) + BEMF ZC 检测 + 速度 PI |
| `Components/comp_resolver.h` | **旋转变压器接口** — 浮点解算 + IQmath 定点解调 (DDS/PLL), 含 comp_iqmath.h |
| `Components/comp_mpu.h` / `comp_mpu_dmp.c` | MPU6050 DMP 算法层 |
| `Components/comp_pfc.h` | **PFC 功率因数校正** — 电流指令 (PfcICmd) + 无桥 (PfcBlIcmd) + RMS² 倒数 |
| `Components/comp_esmo.h` | **增强滑模观测器 (eSMO)** — PLL 锁相环 + 反电势滤波, PMSM/BLDC 无感 FOC |
| `Components/comp_dlog.h` | **数据记录器** — 1ch/4ch 环形缓冲 + 触发 + 预分频 (Dlog1ch/Dlog4ch) |
| `Components/comp_vector.h` | **向量运算库** — 批量 add/sub/mul/dot/mag/absmax/clamp + Vector3 三相便捷结构体 |
| `Components/comp_complex.h` | **复数运算库** — 直角坐标 + 极坐标, 值类型, 矢量旋转 (complex_expj), ISR 安全 |
| `Components/comp_crc.h` | **CRC 校验库** — 5 种多项式 (CRC-8/CRC-8-CCITT/CRC-16/CRC-16-CCITT/CRC-32) 查表法, ISR 安全 |
| `Components/comp_viterbi.h` | **Viterbi 卷积解码器** — K=7 R=1/2, 硬判决+软判决, 滑动窗口回溯, ACS 蝶形运算 |
| `Components/comp_rs.h` | **Reed-Solomon RS(255,239) 编解码** — GF(256) 查表, 系统编码, Berlekamp-Massey + Chien + Forney 译码 |
| `Components/comp_interleaver.h` | **卷积交织器** — Forney 型 B分支×D延迟, 配合 RS 突发纠错, 含解交织器 |
| **App** | |
| `App/app_main.c.tmpl` | App 实现模板 — board_init、ISR、BackgroundTask |
| `App/app_main.h.tmpl` | App 头模板 — 根结构体、配置 POD |
| **Module** | |
| `Module/mod_sfra.h/c` | **SFRA 频响分析仪** — DDS 扰动注入 + DFT 采集 + Bode 增益/相位在线测量 |
| `Module/mod_fcl_ctrl.h/c` | **快速电流环 (FCL)** — dq 旋转坐标系 PI + 交叉解耦 + 反电动势前馈 + 有源阻尼 |
| `Module/mod_pmbus.h/c` | **PMBus 协议栈** — SMBus 2.0 + PMBus 1.3 命令集 (Linear11/16 格式), I2C 从机数字电源通信 |
| **工程** | |
| `Config/params/` | YAML 配置变体 (default.yaml, aggressive.yaml 等) |
| `Config/topologies/` | 拓扑目录 YAML — 每拓扑一个 (buck/boost/forward/flyback/buckboost/sepic/cuk/zeta/buck2/vsi_3ph), 驱动 GUI 拓扑选择器 |
| `Config/projects/` | 工程 YAML — 拓扑+MCU 实例化 (acdc_sixswitch.yaml 等), scaffold 骨架生成输入 |
| `YmaC/` | YAML → C designated initializer 配置注入工具 (Python GUI/CLI) |
| `cmake/` | ARM Clang + GCC + C2000 工具链文件 |

## 3. 项目保障

| 路径 | 用途 |
|------|------|
| [agent.md](agent.md) | 本文件 — AI/人类共读的总纲，完整 OOP 方法论 |
| [docs/debug/LESSONS.md](docs/debug/LESSONS.md) | 调参教训库 (58 条 + 经验模板), git 版本管理, 禁止回退 |

## 3.5 BSP 硬件加速抽象层 🔌

> **Components 层禁止直接 include 平台加速库 (`arm_math.h`, `C2000Ware_dsp.h` 等)。所有硬件加速走 BSP 抽象。**

| BSP 文件 | 抽象内容 | M4F/M7 (HW) | M0+/M3 (SW) | C2000 | 纯C回退 |
|----------|---------|-------------|-------------|-------|--------|
| `BSP/bsp_dsp.h` | sqrt, biquad IIR | CMSIS-DSP FPU SIMD | CMSIS-DSP 软件库 | C2000Ware TMU/CLA | 牛顿迭代 + DFI |
| `BSP/bsp_dsp_fir.h` | 单级 FIR (BspFirInst) + Q15 定点 (BspFirQ15) | CMSIS-DSP arm_fir_f32 | CMSIS-DSP 软件库 | C2000 FPU | 纯C卷积 | 手动覆盖: BSP_DSP_ARCH_Q15 |
| `BSP/bsp_pwm.h` | PWM 不透明句柄 + 物理参数API | bsp_hrtim.c (HRTIM) | — | bsp_c2000_epwm.c (ePWM) | — |
| `BSP/bsp_adc.h` | ADC 校准 + DMA 启动 | bsp_adc_stm32.c | — | bsp_adc_c2000.c | — |

**Components 侧硬件加速宏（`comp_math.h` 分发到 BSP，工程可 `#define` 覆盖后端）：**
```c
MATH_SQRT(x)  → bsp_sqrt_f32  // VSQRT / C2000 TMU __sqrt / 纯C牛顿
MATH_ISQRT(x) → bsp_isqrt_f32 // C2000 CLA CLAisqrt / 纯C 1/sqrtf
MATH_ABS(x)   → fabsf         // FPU VABS 单指令
```
同时 `comp_math.h` 是全库唯一 π/2π float 常量源（`M_PI`/`M_2PI`），定义前 `#undef` 防止系统 `<math.h>` 的 double M_PI 泄漏（软浮点 double 在 M4F/C2000 上是性能灾难）。

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

**平台自动检测 (`BSP/bsp_dsp.h`) — 五级能力分级：**
```c
// BSP_DSP_ARCH: 2=硬件加速 1=软件库 4=Q15定点 3=C2000Ware 0=纯C回退
// BSP_DSP_ARCH_Q15 (4) — 手动覆盖, 用于 Q15 定点 FIR (bsp_dsp_fir.h)
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

**关键原则：** 每个 `#if` 分支的行为语义等价（滤波就是滤波，不是直通）。回退路径不是"后续实现"的占位符——它就在当前版本真实工作。见 [docs/debug/LESSONS.md](docs/debug/LESSONS.md) #34.

## 4. 子系统总览（按文件前缀区分）

| 域 | Component | Devices 子类 | Module | 句柄头文件 |
|----|-----------|-------------|--------|-----------|
| **ADC** | `comp_adc.h/c` | `adc_follower`, `adc_dc_sampler`, `adc_ac_sampler` | `mod_sampler` | `adcs.h` |
| **COM** | `comp_comm.h/c` | `com_uart`, `com_spi`, `com_i2c`, `com_can`, `com_key`, `com_mpu6050`, `com_oled`, `com_ultrasonic`, `com_encoder` | `mod_comm`, `mod_cmd_dispatch`, `mod_serial_proto`, `mod_pmbus` | `comms.h` |
| **GPO** | `comp_gpo.h/c` | `gpo_led`, `gpo_laser`, `gpo_beep`, `gpo_buzzer`, `gpo_fan` | — | `gpos.h` |
| **PID** | `comp_pid.h/c` | `pid_standard`, `pid_cascade`, `pid_p2pd`, `pid_parallel`, `pid_pr`, `pid_qpr` | — | `pids.h` |
| **PWM** | `comp_pwm.h/c` | `pwm_buckboost`, `pwm_half_bridge`, `pwm_full_bridge`, `pwm_interleaved`, `pwm_resonant`, `pwm_svpwm` | `mod_powerctrl` | `pwms.h` |
| **Motor** | `comp_motor.h/c` | `motor_tim` | `mod_motor`, `mod_turn`, `mod_follower`, `mod_fcl_ctrl` | — |

**独立 Component（无 Devices 层, 单头文件 static inline）：**

| Component | 核心结构体 | 用途 |
|-----------|-----------|------|
| `comp_filter.h` | — (biquad DFI) | 一阶低通 + 二阶巴特沃斯低通 (硬件加速走 bsp_dsp.h) |
| `comp_iir.h` | `IirDf22`, `IirDf23`, `Iir2p2z`, `Iir16Cfg/State`, `Iir32Cfg/State` | IIR 数字补偿器 — DF22/DF23/2P2Z float + Q15/Q31 定点 |
| `comp_sgen.h` | `SgenFixed`, `SgenSweep`, `SgenHp1`, `SgenHp2`, `SgenT3D`, `SgenProfile`, `SgenDeadzone` | 7 种信号发生器 — DDS/扫频/探测/Profile/死区测试 |
| `comp_fft_window.h` | (FftWinType 枚举) | 18 种 FFT 窗函数 float + Q31 (fill_q31/apply_q31), 相干增益/ENBW |
| `comp_pi_reg4.h` | `PiReg4Cfg`, `PiReg4State` | 4 状态 PI 调节器 — 设定值滤波 + P + I + 前馈, 双抗积分饱和 |
| `comp_pid_reg3.h` | `PidReg3Cfg`, `PidReg3State` | 3 状态 PID — 反计算抗饱和 + 位置回绕变体, 微分作用在比例输出 |
| `comp_protection.h` | — | 保护框架 — 阈值检测、去抖、分级响应 (Components/protection/ 域) |
| `comp_protection_3lvl.h` | `Prot3LvlDelay` | 三电平逆变器延迟保护 — 主开关立即关断 + 内开关故障消隐延迟关断 (ride-through, 非锁存自重新布防) |
| `comp_pfc.h` | `PfcICmd`, `PfcBLICmd`, `PfcBlIcmd`, `PfcInvRmsSqr`, `PfcInvSqr` | PFC 电流指令, 含无桥桥臂选择 |
| `comp_esmo.h` | `EsmoCfg`, `EsmoState` | eSMO 滑模观测器 — PLL 角度/速度跟踪 |
| `comp_dlog.h` | `Dlog1ch`, `Dlog4ch` | 数据记录器 — 环形缓冲 + 触发 + 预分频 |
| `comp_vector.h` | — (float* 数组), `Vector3` | 向量批量运算 + 三相便捷结构体 |
| `comp_complex.h` | `Complex32` | 复数运算 (直角+极坐标) — add/sub/mul/div/mag/phase/conj/expj |
| `comp_crc.h` | `crc8/16/32_table[]` | CRC 校验 — 5 种多项式查表法, init 参数链式分片 |
| `comp_viterbi.h` | `ViterbiState` | Viterbi 卷积解码器 — K=7 ACS 蝶形 + 滑动窗口回溯 |
| `comp_rs.h` | `RsCfg`, `RsState` | Reed-Solomon RS(255,239) — GF(256) BM+Chien+Forney 译码 |
| `comp_interleaver.h` | `InterleaverCfg`, `Interleaver` | 卷积交织器 — Forney 型 B分支×D延迟, 配合 RS 纠错 |
| `comp_bldc_instaspin.h` | `BldcInstaSpinCfg`, `BldcInstaSpinState` | InstaSPIN-BLDC — 3阶段启动 + BEMF ZC + 速度 PI |
| `comp_aci_se.h` | `AciSeConst`, `AciSe` | ACI 转差法转速估计 — 磁链角微分 + 转差计算 (与 comp_aci_fe 配对) |
| `comp_mod6.h` | `Mod6Cnt` | 模 6 换相计数器 — BLDC 六步换相步进 (0→5→0) |
| `comp_impulse.h` | `Impulse` | 脉冲发生器 — 每 Period 采样输出满幅脉冲 (0x7FFF) |
| `comp_sogi_fll.h` | `SogiFll`, `SogiFllOsgCoeff`, `SogiFllLpfCoeff` | SOGI 锁相环 FLL 变体 — SOGI-QSG + 频率锁定环, 自适应电网频率漂移 |
| `comp_power_meas.h` | `PowerMeas`, `EnergyAccu`, `PowerMeas3Ph` | 电力测量 — Vrms/Irms/P/Q/S/PF/相位角 + 能量脉冲积分 (残余结转) + 三相聚合 (总功率/线电压/电流矢量和) |
| `comp_power_fund.h` | `PowerFund` | 基波电力分析 — 同步正交相关解调, 基波 Vrms/Irms/P/Q/THD (IEC 62053) |
| `comp_power_goertzel.h` | `PowerGoertzel` | Goertzel 逐谐波频谱 (H1..H50) + THD — 整数周期窗口谐振器, 无需窗函数 |
| `comp_power_calib.h` | `PowerCalibPhase` | 结果级校准 POD — 死区减法 (保符号对称死区), 即 TI NV 持久化结构体 |
| `comp_power_event.h` | `PowerEvent` | 电压事件检测 — 暂降/暂升/中断状态机 + 事件计数/时长 (滞回 + 交叉检测) |
| `comp_arc_detect.h` | `ArcDetect` | 光伏电弧检测 — FFT 频带能量 2 子带加权 + 单频干扰滤除 + dB 阈值判定 |
| `comp_pid_nl.h` | `NlPidCfg`, `NlPidState` | 非线性 PID — P/I/D 各通路独立幂律整形 (α/δ/γ), 强鲁棒控制 |
| `comp_tcm.h` | `TcmCapture` | 自动调参 TCM — 触发式阶跃响应捕获 (预触发环) + IAE/ISE/ITAE 准则 |
| `comp_resolver.h` | `Resolver`, `ResolverFixedCfg`, `ResolverFixedState` | 旋变接口 — 浮点解算 + IQmath DDS/PLL 定点解调 |
| `comp_math.h` | — | 数学工具 — 唯一 π/2π float 常量源 (M_PI/M_2PI) + 硬件加速宏 (MATH_SQRT/MATH_ISQRT/MATH_ABS) + 限幅/绝对值/死区/线性映射 |
| `comp_error.h` | — | 统一错误码 bitmask — ERROR_SET/CLEAR/IS_SET 宏 |

> 文件按子系统归入子目录：`Components/<域>/`、`Devices/<域>/`、`Module/<域>/`。文件前缀仍是域标识，子目录与之一致（如 `Components/pid/comp_pid.h`、`Devices/pwm/pwm_svpwm.h`、`Module/motor/mod_motor.h`）。
> 每个子目录含 `MANIFEST.yaml` 自描述（id/files/depends），供 `YmaC/scaffold.py` 做依赖解析和项目骨架生成。完整文件→目录映射见 [docs/debug/build-toolchain-design.md](docs/debug/build-toolchain-design.md) 第一节。

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

创建新的驱动族（如 LED、Motor、Sensor）时，按以下文件布局（子系统子目录）：

```
Components/<域>/comp_xxx.h/c  — XxxBase 结构体, XxxOps typedef, 分发函数
Components/<域>/MANIFEST.yaml — 子系统自描述 (id/files/depends)
Devices/<域>/xxx_variant.h/c  — 具体子类 × N (xxx_gpio, xxx_pwm, xxx_i2c...)
Devices/<域>/xxxs.h           — extern 全局句柄声明
Module/<域>/mod_xxx.h/c       — (可选) 业务模块，组合多个 Device
App/app_main.c                — #include "xxxs.h", 使用句柄
```

> `<域>` 与文件前缀一致：adc / comm / gpo / pid / pwm / motor / dsp / power / math / codec / sensor。

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
2. 确保 `BSP/container_of.h` 和 `Components/comp_math.h` 加入 include path
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

各子系统的继承树、文件清单、依赖、和具体复用示例见下方。每个子系统对应一组文件前缀，文件按子系统归入 `Components/<域>/`、`Devices/<域>/`、`Module/<域>/` 子目录。文件→目录映射见 [docs/debug/build-toolchain-design.md](docs/debug/build-toolchain-design.md) 第一节。

### 8.1 ADC 子系统 — 多通道采样

**继承树：**
```
AdcBase (虚表 + 名称 + DMA 缓冲区指针 + 位置偏差)
├── AdcFollower   — 8 路红外循迹传感器 (二值化 + 独热码 + 位置映射)
├── AdcDcSampler  — 通用直流采样器 (N 通道 EMA 滤波 + k·raw+b 线性校准)
└── AdcAcSampler  — 三相交流采样器 (差分采样 + 三相重构 + RMS + Vdc)
```

**AdcOps 虚表（3 必须 + 2 可选）：** start_dma(必须) / read_ch(必须) / process(必须) / get_sum2(可选) / get_ch_bin(可选)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h`, STM32 HAL

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
├── ComUltrasonic — 超声波测距 (触发→接收→解码)
└── ComEncoder   — 位置编码器 (BiSS-C/Endat22/SinCos/T-Format/PTO, 协议分发 + 位置解算 + 速度估计)
```

**CommOps 虚表（4 必须 + 2 可选）：** send(必须) / bgn(必须) / read(必须) / avail(必须) / is_ok(可选) / reset(可选)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h`, `Components/comp_mpu.h`, STM32F1 HAL

**ComEncoder 位置编码器 (CommBase 子类)：**

> **来源:** TI controlSUITE position_manager, 翻译为 HardC 纯C 版本

**支持协议:** BiSS-C (RS485 双向, MA+SLO+CRC6) / Endat22 (RS485 双向, 命令帧+MRS码) / SinCos (模拟 1Vpp 差分, 正余弦插值) / T-Format (串行单向, 纯接收) / PTO (脉冲序列 ABZ+UVW, 正交计数)

**关键 API:** `encoder_init(cfg)` → `encoder_read_position()` (ISR 热路径) → `encoder_update(dt)` (主循环, 速度估计+诊断) → `encoder_get_angle/get_velocity/get_error_count/is_ok`

**EncoderCfg POD:** proto / bits_single / bits_multi / freq_hz / timeout_us / use_crc

**Encoder Instance:** CommBase 父类 + 位置缓存 (raw_position/single_turn/multi_turn/angle_rad/velocity) + 错误状态 (connected/crc_error/timeout_error/error_count) + union proto_data (按协议特有数据)

**依赖:** `Components/comp_comm.h`, `<stdint.h>`, `<stdbool.h>`

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

**依赖：** `BSP/container_of.h`, `Components/comp_math.h`, `<math.h>`

### 8.5 PWM 子系统 — 电力电子拓扑

**继承树：**
```
PwmBase (虚表 + 模式 + 通道数 + 频率 + 占空比限幅 + 运行状态)
├── PwmBuckBoost   — 单路/多相 Buck/Boost (相位参数化 N=1..3, BUCK/BOOST/BUCKBOOST duty law 入 Device, 可选同步整流)
├── PwmHalfBridge  — 半桥互补 PWM (中心对齐 + 死区)
├── PwmFullBridge  — 全桥移相 PWM (A/B 两腿 + 移相角控制功率)
├── PwmInterleaved — 多相交错并联 PWM (N 相均匀错相 360°/N)
├── PwmResonant    — 谐振变频 PWM (50% 固定占空比 + 变频控制)
├── PwmSvpwm       — 六开关 SVPWM (三相逆变桥, 7段对称 + 5段 DPWM 断续调制)
└── PwmWpt         — 无线充电线圈驱动 (单相半桥, 占空比下限 + VB_LIMIT_BY_DUTY + 频率分频)
```

**PwmOps 虚表（4 必须 + 3 可选）：** start(必须) / stop(必须) / set_duty(必须) / set_freq(必须) / set_deadtime(可选) / set_phase(可选) / emergency_stop(可选)（`pwm_set_*` 包装对 NULL 安全检查, 可选可省略不实现）

**BSP 物理参数 API (推荐)：** `bsp_pwm_config_ch`, `bsp_pwm_set_duty_f`, `bsp_pwm_set_freq_hz`, `bsp_pwm_set_deadtime_ns`, `bsp_pwm_set_phase_deg`, `bsp_pwm_set_complementary`, `bsp_pwm_isr`

**依赖：** `BSP/container_of.h`, `BSP/bsp_pwm.h`, `BSP/bsp_dsp.h`, `<math.h>`

### 8.6 Motor 子系统 — 直流电机

**继承树：**
```
MotorBase (虚表 + 名称 + ops)
└── MotTim — TIM4 PWM 双通道 + AB 相编码器 4 倍频鉴相 (motor_tim.h/c)
```

**MotorOps 虚表（3 必须）：** write(必须) / encode(必须) / read(必须)

**关键特性：** `inv` 模式 — 驱动层取反 DIR + 编码器方向, 应用层看到对称行为 (电机面对面安装). Lesson #28 适用.

**Module 层：** `mod_motor.h/c` (MotApp 状态机) / `mod_turn.h/c` (TurnCtrl 转弯) / `mod_follower.h/c` (Follower 循迹)

**依赖：** `BSP/container_of.h`, `Components/comp_math.h`, STM32F1 HAL

#### comp_bldc_instaspin.h — InstaSPIN-BLDC 无传感器方波驱动

> **来源:** TI InstaSPIN-BLDC 算法概念 (SPRA590/SPRA695/SPRABQ7), 解绑自 ROM 实现, 翻译为 HardC 纯C float inline 版本
> **新增日期:** 2026-08-12

**三阶段启动 + 闭环换向:**
```
ALIGN (强制对齐) → OPENLOOP (开环加速, V/f 控制) → CLOSED (BEMF ZC 换向 + 速度PI)
    ↓                      ↓                              ↓
  FAULT ←———————————————— (堵转 / 过流) ——————————————————┘
```

**6 步换向真值表 (120° 导通, 方波驱动):** 每步一相 PWM+ (上管调制), 一相 PWM- (下管导通), 一相悬空 (高阻态).

**核心结构体:**
- `BldcInstaSpinCfg` — 配置 POD (pole_pairs/pwm_freq/min_speed/max_speed/align_time/align_duty/startup_accel/kp_speed/ki_speed/zc_blanking_us/zc_filter_cnt/过流阈值/堵转超时)
- `BldcInstaSpinState` — 运行时状态 (换向步/占空比/速度估计/ZC 检测/速度 PI 积分器/开环加速/故障码)

**关键 API (全部 static inline, ISR 安全):**
- `bldc_instaspin_cfg_default(cfg)` / `bldc_instaspin_init(st)` — 初始化, 进入 ALIGN 阶段
- `bldc_instaspin_run(st, cfg, v_bus, va, vb, vc, i_dc, dt)` — ISR 每控制周期调用, 驱动全状态机
- `bldc_instaspin_get_commutation_step/get_duty/get_speed/get_motor_state/get_fault_code(st)` — 访问器
- `bldc_instaspin_get_phase_state(st, &pa, &pb, &pc)` — 获取三相开关状态 (+1=PWM+, -1=PWM-, 0=悬空)
- `bldc_instaspin_clear_fault(st)` — 故障清除, 回到 ALIGN 重新启动

**ZC 检测三重抗噪:** (1) 换向后 blanking 空白窗口避开续流振铃, (2) 滞环阈值 (2% Vbus) 过滤小幅噪声, (3) zc_filter_cnt 连续确认窗口消除偶发抖动.

**依赖:** `<math.h>`, `<stdbool.h>`

### 8.7 StepMotor 子系统 — 步进电机

> **来源:** 既有项目经验（三版步进电机对比 + Bug 清单）
> **新增日期:** 2026-08-12

**继承树：**
```
StepMotorBase (虚表 + 相序表 + 软限位 + 加减速状态)
└── StepMotor — 4 相步进电机, BSP 函数指针注入, DIR+PUL+ENABLE (motor_step.h/c)
```

**StepMotorOps 虚表（6 必须）：**
set_rate(必须) / set_steps(必须) / get_steps(必须) / set_ramp(必须) / set_phase_mode(必须) / emergency_stop(必须)

**BSP 抽象：** `BSP/bsp_step_motor.h` — `BspStepDir` / `BspStepPul` ctx 结构体, `bsp_step_dir_set()` / `bsp_step_pul_set_period()` / `bsp_step_write_phase()` 函数

**相序表可注入：** 全步进 4 拍 (A→C→B→D) / 半步进 8 拍 (A→AC→C→...), 微步进需驱动芯片硬件支持

**关键约束 (Bug 规避):**
- 状态全在结构体成员, 禁止 `static` 局部变量
- 速度控制必须写定时器 LOAD 寄存器 (period 不能是死字段)
- BSP 注入的 ctx 必须被实际使用 (禁止 `(void)ctx`)
- 必须有软限位 + 加减速占位 + 锁止自动关相

**Module 层：** `mod_balance.h/c` (球板平衡, 步进电机 + PID + 摄像头坐标)

**依赖：** `Components/comp_step_motor.h/c`, `BSP/bsp_step_motor.h`, `BSP/container_of.h`, `Components/comp_math.h`

### 8.8 独立 Component（无 Devices 层）

> **来源:** controlSUITE 算法移植（Phase B/C, 2026-08-12）
> 这些是纯算法 Component，没有对应 Devices 子类。全部 static inline，零 malloc。

#### comp_pfc.h — PFC 功率因数校正

**核心结构体：**
```
PfcICmd       — 标准 PFC 电流指令 (Vcmd × VinvSqr × VacRect × VmaxOverVmin)
PfcBLICmd     — 升压开关电流指令 (含占空比前馈)
PfcInvRmsSqr  — 输入 RMS² 倒数 (带最小值限制)
PfcInvSqr     — 带 HALF_PI 缩放的平方倒数
PfcBlIcmd     — 无桥 PFC 电流指令 (正/负半周互斥, 两路输出)
```

**依赖：** `<math.h>`

#### comp_esmo.h — 增强滑模观测器 (eSMO)

**六阶段算法：** 电流估计 → 滑模控制(sat) → 反电动势滤波 → PLL 鉴相 → PI+VCO → 角度归一化

**调用方式（ISR 中每控制周期）：**
```c
esmo_run(&obs, &cfg, v_alpha, v_beta, i_alpha, i_beta);
float theta = esmo_get_theta(&obs);
float speed = esmo_get_speed(&obs);  // 电角速度 rad/s
```

**相比基础 SMO (comp_smo.h) 的改进：** PLL 替代 arctan (角度更平滑) + 反电动势低通滤波 (减少抖振) + 速度直接由 PLL 输出

**依赖：** `<math.h>`

#### comp_dlog.h — 数据记录器

**两种变体：**
- `Dlog1ch` — 单通道环形缓冲 + 预分频 + 触发后 oneshot 捕获至满
- `Dlog4ch` — 四通道交替存储 + 自动触发电平越限检测 (上升/下降沿)

**使用示例（ISR + 后台）：**
```c
// ISR 内: dlog4ch_capture(&log, vout, iout, vin, temp);
// 后台: for (int i = 0; i < dlog4ch_count(&log); i++)
//          printf("%.4f\n", dlog4ch_read(&log, 0, i));
```

**依赖：** `<stdbool.h>`, `<stddef.h>`

#### comp_vector.h — 向量/矩阵批量运算

**基本向量运算（操作 float* + len）：** vec_add/sub/mul/scale/dot/mag/absmax/argmax/mean/fill/copy/clamp_f32

**Vector3 三相便捷结构体：** `{float a, b, c}` + vec3_add/sub/scale/dot/mag/cross/clamp

**三后端策略：** 默认纯C for循环 / BSP_DSP_ARCH≥2 → CMSIS-DSP / BSP_DSP_ARCH≥3 → C2000Ware CLA

**依赖：** `<math.h>`

#### comp_complex.h — 复数运算库 (直角坐标 + 极坐标)

> **来源:** TI controlSUITE vcu/ComplexMath (complex_math.h)

**核心类型：** `Complex32` — 值类型 `{float re, im}`, 按值传递/返回, ARM EABI 硬浮点 ABI 走 VFP 寄存器, 零栈开销, ISR 友好

**构造宏：** `COMPLEX32(re, im)` / `COMPLEX32_ZERO`

**基本运算 (static inline)：**
- 四则: `complex_add/sub/mul/div(a, b)` — 除法用共轭分母避免不稳定
- 属性: `complex_mag(a)` / `complex_mag_sq(a)` (避免 sqrt) / `complex_phase(a)` (atan2, ∈[-π,π])
- 变换: `complex_conj(a)` / `complex_neg(a)` (180°旋转) / `complex_normalize(a)` (单位化)

**矢量旋转核心（DDS/锁相环）：** `complex_expj(theta)` — e^(jθ) = cosθ + j·sinθ, 用法: `rotated = complex_mul(v, complex_expj(theta))`

**极坐标构造：** `complex_polar(mag, phase)` — mag × e^(j×phase)
**标量乘：** `complex_scale(a, s)` — (a.re×s, a.im×s)

**依赖：** `<math.h>` (cosf/sinf/sqrtf/atan2f)

#### comp_crc.h — CRC 校验库 (5 种多项式, 查表法)

> **来源:** TI controlSUITE vcu/CRC (vcu_crc.h)

**5 种多项式 (CrcPoly 枚举)：**
| 多项式 | poly | 位宽 | 生成算法 | 用途 |
|--------|------|------|---------|------|
| CrcPoly_8 | 0x07 | 8-bit | MSB-first | 通用 8-bit CRC |
| CrcPoly_8_CCITT | 0x8D | 8-bit | MSB-first | Qi 无线充电标准 CRC |
| CrcPoly_16 | 0x8005 | 16-bit | LSB-first (反射) | CRC-16/IBM-ARC |
| CrcPoly_16_CCITT | 0x1021 | 16-bit | MSB-first (非反射) | CRC-16-CCITT |
| CrcPoly_32 | 0x04C11DB7 | 32-bit | LSB-first (反射) | Ethernet/ZIP CRC-32 |

**核心函数 (全部 static inline, 查表法, ISR 安全)：**
- `crc8_calc(data, len, init, poly)` — init 支持链式分片 (常用: 0x00 或 0xFF)
- `crc16_calc(data, len, init, poly)` — init: CRC-16 通常 0x0000, CRC-16-CCITT 通常 0xFFFF
- `crc32_calc(data, len, init)` — Ethernet 标准: init=0xFFFFFFFF, final XOR=0xFFFFFFFF (调用者自行处理)

**依赖：** `<stdint.h>`

#### comp_viterbi.h — Viterbi 卷积码解码器 (K=7 R=1/2)

> **来源:** TI controlSUITE vcu/Viterbi (vcu_viterbi.h), 参考 Phil Karn's KA9Q Viterbi

**编码器参数：** 约束长度 K=7, 码率 R=1/2, 生成多项式 G0=0x6D (133₈) G1=0x4F (171₈) — NASA/CCSDS 标准卷积码, 用于卫星通信 + Qi 无线充电 ASK/FSK 链路

**预计算表：** `viterbi_output_table[128]` — 64 状态 × 2 输入 bit 的期望编码输出, 索引 `(state << 1) | input_bit`

**核心结构体 `ViterbiState`：**
- `path_metrics[64]` — 64 状态的路径度量 (uint16_t, 状态0起始=0, 其余=max)
- `traceback[64 × 35]` — 幸存路径历史矩阵 (VITERBI_TRACEBACK_DEPTH=35)
- `tb_pos` / `step_count` — 回溯位置 + 归一化计数

**ACS 蝶形运算 (热路径)：**
- `viterbi_update_hard(me, sym_a, sym_b)` — 硬判决, 分支度量=Hamming 距离 (2-bit XOR + popcount)
- `viterbi_update_soft(me, llr_a, llr_b)` — 软判决, 分支度量=Manhattan 距离 (llr ∈ [0,255], 0=strong 0, 255=strong 1)

**滑动窗口回溯：**
- `viterbi_traceback(me, out, max_len)` — 从最小度量状态回溯 35 步, 翻转 bits 输出正向解码
- `viterbi_decode_bit(me)` — 简化单 bit 解码 (不足回溯深度返回 -1)

**归一化:** 每 64 步 (RENORM_INTERVAL) 所有度量减最小值, 防 uint16_t 溢出

**典型用法：**
```c
ViterbiState vt;
viterbi_init(&vt);
// ISR 中每收到一对符号:
viterbi_update_hard(&vt, rx_bit_a, rx_bit_b);   // 或软判决
// 回溯解码:
int n = viterbi_traceback(&vt, decoded, sizeof(decoded));
```

**依赖：** `<stdint.h>`, `<string.h>` (memset)

#### comp_pi_reg4.h — 4 状态 PI 调节器

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3/pi_reg4.h, 扩展前馈 + 设定值滤波
> **新增日期:** 2026-08-12

**四路径算法:**
1. **设定值滤波** — 一阶低通 (可选, sp_fc > 0 生效), 首帧直通防延迟; alpha = 2π·fc·dt / (1 + 2π·fc·dt)
2. **P 路径** — Kp × (sp_filtered - fbk), 即时偏差响应
3. **I 路径** — Σ Ki × dt × error, 含双抗积分饱和: 边界检测 (上一拍输出在限幅边界且误差方向朝外 → 冻结) + clamping 回退 (输出达限幅且积分项同向 → 回退积分)
4. **FF 路径** — Kff × sp_raw (不经滤波器), 大范围跟踪加速

**核心结构体:** `PiReg4Cfg` (配置: kp/ki/kff/dt/out_max/out_min/sp_fc) + `PiReg4State` (状态: integral/sp_filtered/last_output/initialized)

**关键 API (全部 static inline):**
- `pi_reg4_cfg_default()` → 返回安全默认配置 (Kp=1, Ki=0, Kff=0, sp_fc=0 直通)
- `pi_reg4_init/reset(me)` — 初始化/重置积分器 (保留滤波状态)
- `pi_reg4_run(me, cfg, setpoint, feedback)` — ISR 热路径, 返回限幅后输出

**依赖:** `<stdbool.h>`

#### comp_pid_reg3.h — 3 状态 PID 调节器 (反计算抗饱和)

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3/pid_reg3.h
> **新增日期:** 2026-08-12

**与 comp_pi_reg4 的区别:** reg3 用反计算 (back-calculation) 抗饱和 — 输出被限幅时把饱和差 `SatErr = Out - OutPreSat` 反馈回积分器, 从根上抑制 windup (pi_reg4 用冻结 + clamping)

**两种变体:**
1. **标准** `pid_reg3_run` (对应 TI `PID_REG3_MACRO`) — `Err = Ref-Fdb`; `Up = Kp×Err`; `Ui += Ki×Up + Kc×SatErr`; `Out = sat(Up+Ui)`; `SatErr = Out-OutPreSat`. 注意积分作用在**比例输出**而非误差
2. **位置** `pid_reg3_run_pos` (对应 TI `PID_REG3_POS_MACRO`) — 误差在 ±0.5 处回绕 (角度归一化 [0,1] 跨越边界时取短路径), 增加微分 `Ud = Kd×(Up-Up1)`

**核心结构体:** `PidReg3Cfg` (kp/ki/kc/kd/out_max/out_min) + `PidReg3State` (ref/fdb/err/up/ui/ud/up1/out_pre_sat/out/sat_err)

**关键 API (全部 static inline):**
- `pid_reg3_cfg_default()` — TI 默认值 Kp=1.3 / Ki=0.02 / Kc=0.5 / Kd=1.05 / ±1.0
- `pid_reg3_init/reset(me)` — 初始化/重置积分器 (保留比例/微分状态)
- `pid_reg3_run(me, cfg, ref, fdb)` — 标准变体
- `pid_reg3_run_pos(me, cfg, ref, fdb)` — 位置变体

**依赖:** 无 (纯 float)

#### comp_aci_se.h — 异步电机转差法转速估计器

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3 (aci_se.h, aci_se_const.h)
> **新增日期:** 2026-08-12

**与 comp_aci_fe 配对:** aci_fe 估计转子磁链 → aci_se 从磁链 + 电流估计转速 (转差法, 感应电机无传感器转速估计)

**算法 (全标幺 pu):**
1. **转差速度** `WSlip = K1×(PsiDr×IQs − PsiQr×IDs)/|Psi|²` (低磁链保护, 防除零)
2. **同步转速** `WSyn = K2×ΔThetaFlux` — 磁链角差分, 仅在角度 0.20~0.80 线性区有效 (0/1 边界回绕差分会跳变 → 保持上一拍)
3. **低通滤波** `WPsi = K3×WPsi + K4×WSyn` (抑制微分噪声)
4. **转子转速** `WrHat = WPsi − WSlip`, 饱和 [-1,1] pu
5. **转速输出** `WrHatRpm = BaseRpm × WrHat`

**系数计算 aci_se_const_calc:** Tr=Lr/Rr; Tc=1/(2π·fc); Wb=2π·fb; K1=1/(Wb·Tr); K2=1/(fb·Ts); K3=Tc/(Tc+Ts); K4=Ts/(Tc+Ts)

**关键 API (全部 static inline):**
- `aci_se_const_calc(&cfg)` — 物理参数 → 系数
- `aci_se_init(me, &cfg, base_rpm)` — 初始化
- `aci_se_run(me, i_qs_s, i_ds_s, psi_dr_s, psi_qr_s, theta_flux)` — ISR 热路径, 返回估计转速 (rpm)
- `aci_se_reset(me)` — 重置

**依赖:** 无 (纯 float)

#### comp_impulse.h — 脉冲发生器

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3/impulse.h
> **新增日期:** 2026-08-12

每 Period 个采样周期输出一个满幅脉冲 (0x7FFF), 其余输出 0. 用于注入阶跃/冲激测试信号, 配合频响分析 (SFRA) 观测系统动态响应. 输出数值与 comp_dlog 满刻度约定一致.

**关键 API (static inline):**
- `impulse_init(me, period)` — period 为脉冲间隔采样数
- `impulse_tick(me)` — 每采样周期调用, 返回 0 或 0x7FFF

**依赖:** `<stdint.h>`

#### comp_mod6.h — 模 6 换相计数器

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3/mod6_cnt.h
> **新增日期:** 2026-08-12

BLDC 六步换相: 触发有效时换相步 0→1→...→5→0 循环 (电角度每 60° 换相一次). 触发输入 Q0 >0 有效.

**关键 API (static inline):**
- `mod6_init(me)` — 清零
- `mod6_tick(me, trig)` — 触发沿调用, 返回当前换相步 0~5 (可索引六步换相电压矢量表)

**依赖:** `<stdint.h>`

#### comp_sogi_fll.h — SOGI 锁相环 FLL 变体 (频率锁定环)

> **来源:** TI C2000Ware Digital Power SDK libraries/spll/include/spll_1ph_sogi_fll.h
> **新增日期:** 2026-08-12

与 comp_pll.h 的 SogiPll (SOGI-PLL, 固定标称频率) 的区别: 增加频率锁定环 (FLL), 用正交输出误差 `ef2 = −(u−u_α)·u_β·γ·dt` 驱动频率积分器 `x3`, `w_dash = wc + x3` 实时跟踪电网频率漂移, 并每拍用自适应频率重算 SOGI 双线性系数. 适用于弱电网/频率漂移场景 (柴油发电机、微电网、变速发电机接口).

**关键 API (static inline):**
- `sogi_fll_init(me, grid_freq_hz, isr_freq_hz, lpf_b0, lpf_b1, k, gamma)` — 配置 (k=SOGI 阻尼典型 √2, γ=FLL 收敛增益)
- `sogi_fll_run(me, ac_value)` — 每采样周期调用, 输出 theta/cosine/sine/fo (锁相角/正交量/估计频率 Hz)
- `sogi_fll_reset(me)` — 复位状态, 保留自适应频率 w_dash
- `sogi_fll_coeff_calc(me)` — 初始化用系数计算 (清零 FLL 积分器); run() 内部走 `coeff_recalc` 不清零

**依赖:** `<math.h>`

#### comp_power_meas.h — 电力测量 (真有效值/功率/能量积分)

> **来源:** TI C2000Ware Digital Power SDK
>   power_measurement/include/power_meas_sine_analyzer.h +
>   energy-metrology_library/energy_metrology_f28p55 (metrology_background/calculations)
> **新增日期:** 2026-08-12

单相功率计量核心: 逐采样累加 v²/i²/(v·i)/(v_quad·i), 无功经电压历史环形缓冲延迟 1/4 周期 + 插值得 90° 正交电压; 每窗口结算 Vrms/Irms/P/Q/S/PF/相位角. 能量积分按阈值脉冲输出 (模拟表计 LED/刻度盘), 残余结转防精度丢失, 导入/导出分离. DC 去除用一阶高通 y+=(x−y)/16384 消除 ADC 直流偏置.

**关键 API (static inline):**
- `power_meas_init(me, sample_rate, threshold, window_n)` — window_n = 一周期采样数 (quad_delay 自动 = N/4)
- `power_meas_sample(me, v, i)` — ISR 逐采样累加 (含过零测频)
- `power_meas_update(me)` — 每 N 采样结算一次 Vrms/Irms/P/Q/S/PF/φ 并清累加器
- `power_meas_dc_filter(state, x)` — 一阶 DC 去除
- `energy_accu_init(me, threshold_wh)` — 脉冲阈值 (如 0.1 Wh)
- `energy_accu_integrate(me, power_w, dt_s)` — 每窗口调用, 返回本次脉冲数

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_power_fund.h — 基波电力分析 (同步正交相关解调)

> **来源:** TI C2000Ware Digital Power SDK
>   libraries/energy-metrology_library/energy_metrology_f28p55
>   (metrology_background.c 正交相关累加 + metrology_calculations.c
>    calculateFundamentalRMSVoltage/ActivePower/ReactivePower + THD)
> **新增日期:** 2026-08-12

窄带同步解调, 只在测得电网频率上提取基波 — 对谐波污染免疫. 每采样相位参考步进 `phase += 2π·f_ref·dt` (连续不随结算复位), 以 sinθ/cosθ 正交参考做相关累加 `v_in += v·sinθ; v_cos += v·cosθ; i_in += i·sinθ; i_q += i·(−cosθ)` (TI 无功符号: 滞后电流 → Q>0). 窗口结算 (N 样本): `V_mag2 = sqrt((v_in/N)²+(v_cos/N)²)`, `FRMS = V_mag2·√2`, THD = sqrt(RMS²−FRMS²)/FRMS·100 (RMS 由宽频测量输入).

功率按电压复矢量投影 — **旋转不变**: `P = 2·(v_in·i_in − v_cos·i_q)`, `Q = 2·(v_cos·i_in + v_in·i_q)`. 参考帧与电网电压间的任意相位偏移自动消除, ADC 无需与过零同步; 与 TI 计量库的 VoltagePure 相位对齐数学等价.

**关键 API (static inline):**
- `power_fund_init(me, sample_rate, f_ref, window_n)` — window_n = 结算窗样本 (典型 4 周期)
- `power_fund_sample(me, v, i)` — ISR 每采样调用 (相关前先用当前相位, 再步进参考 — 避免首样本错相)
- `power_fund_update(me, rms_v, rms_i)` — 窗口满结算, 返回 1=有新结果; 清累加器但相位参考保持连续
- `power_fund_set_freq(me, f_hz)` — 由电网频率估计 (过零/锁相环) 每窗更新, 抑制窗泄漏
- `power_fund_reset(me)` — 清零累加器与相位参考

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_power_goertzel.h — Goertzel 逐谐波频谱 (H1..H50 + THD)

> **来源:** TI C2000Ware Digital Power SDK
>   libraries/energy-metrology_library/energy_metrology_f28p55
>   (metrology_calculations.c goertzelMagnitude + USE_GOERTZEL_THD 分支)
> **新增日期:** 2026-08-12

对第 h 次谐波跑 2 阶 Goertzel 谐振器, 逐次输出幅值谱 H1..H50: `ω_h = 2π·h·grid_freq/sample_rate`, `coeff = 2·cos(ω_h)`, `q0 = coeff·q1 − q2 + x[k]` (窗口 N 点按时间序); DFT bin `re = q1 − q2·cos(ω_h)`, `im = q2·sin(ω_h)`, 峰值幅值 `A_h = 2·sqrt(re²+im²)/N`. 窗口 = 整数个电网周期 (bin 精确落点) → 无需窗函数; THD = `sqrt(Σ_{h=2..50} A_h²)/A_1 × 100`. TI 的 2048 点 FFT 分支 (fpu_rfft) 为 C2000 硬件绑定, 不移植 — Goertzel 在任意平台等效. 与 comp_power_fund.h (基波相关解调) 互补: 本组件给逐次谐波频谱, fund 给基波功率.

**关键 API (static inline):**
- `power_goertzel_window_n(cycles, sample_rate, grid_freq)` — 整数周期窗口样本数 N (如 4·2000/50 = 160)
- `power_goertzel_init(me, buffer, sample_rate, grid_freq, window_n)` — 缓冲由调用者提供 (零 malloc), 长度 ≥ N
- `power_goertzel_sample(me, x)` — ISR 逐采样写环形缓冲
- `power_goertzel_analyze(me, mag[], max_h)` — 逐谐波谐振, 返回 1=缓冲满且已计算; mag[0]=0, mag[h]=第 h 次峰值幅值

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_power_calib.h — 结果级校准 POD (PowerCalib + 死区减法)

> **来源:** TI C2000Ware Digital Power SDK
>   libraries/energy-metrology_library/energy_metrology_f28p55
>   (metrology_nv_structs.h calibrationData + metrology_calibration.h
>    applyCalibrationPhase: deadband = mean·scale, <offset → 0, 否则 −offset)
> **新增日期:** 2026-08-12

校准在"结果"上做死区减法, 不是逐采样: `result = mean·scale`; `|result| < offset → 0`; 否则朝零方向减 offset. 死区消除小信号偏置 (噪声/串扰残余), 保证空载读数归零. **死区保符号**: 有功/无功/基波功率是有符号量 (馈出/容性为负), TI 的字面死区 (`r < offset → 0`) 会把负读数误归零, 本组件改为对称死区 `|r| < offset → 0`, 真实负值按原符号直通 — 有意偏离 TI, 头注释已记录.

**NV 持久化:** 本 POD 即 TI 的可持久化结构体 — TI 只持久化校准数据 (结构体魔数 0x59 + 校准初始化标志 0xABCD, 无 CRC), **能量计数是 RAM-only 不掉电保持**. flash 读写 + 完整性校验 (可选复用 comp_crc.h) 是应用层职责. 相位偏移 (TI 用 256 项 FIR 系数表, 采样率绑定) 不做 — 相位由测量层 PowerMeas 的 quad_delay 折算.

**关键 API (static inline):**
- `power_calib_init(me)` — 恒等默认 (scale=1, offsets=0), 未校准时原值直通
- `power_calib_deadband(raw, scale, offset)` — 对称死区共用助手 (保符号)
- `power_calib_apply_vrms/irms/frms_v/frms_i/p/q/fp/fq(me, raw)` — 8 个结果校准: Vrms/宽频电流/基波电压/基波电流/宽频有功/宽频无功/基波有功/基波无功

**依赖:** `<math.h>`

#### comp_power_event.h — 电压事件检测 (暂降/暂升/中断状态机)

> **来源:** TI C2000Ware Digital Power SDK
>   libraries/energy-metrology_library/energy_metrology_f28p55
>   (metrology_calculations.c checkSagSwellEvents + cyclePhaseDP 逐周期 RMS)
> **新增日期:** 2026-08-12

逐周期 RMS 判据: `sag_start = Vnom·sag_pct` (默认 0.80), `sag_stop = sag_start + hysteresis`; `swell_start = Vnom·swell_pct` (默认 1.20), `swell_stop = swell_start − hysteresis`; `min_sag_v` (默认 100V, 须 < sag_start) → 中断. 滞回防止阈值边界抖振.

状态机每带一个 tick 助手 (ONSET/CONTINUING 共用): NORMAL 检测到越限即入对应 ONSET 并计入首周期 (active_duration=1); 每带推进顺序 = **深化 → 交叉 → 滞回恢复 → 保持** (交叉检测先于滞回恢复, 捕捉跃入对侧带的单周期尖峰). 计数语义: 恢复/交叉的边界周期不计入时长但计入极值; 深化为中断时过渡周期计入中断时长 (成为中断首周期), 暂降时长定格于深化前. 每个事件完成时 latch `sag_duration/swell_duration/int_duration` + `event_min_v/event_max_v`, 直到下一次事件覆盖.

**关键 API (static inline):**
- `power_event_init(me, v_nominal, sag_pct, swell_pct, hysteresis_v, min_sag_v)` — 计算 start/stop 阈值
- `power_event_sample(me, v)` — ISR 逐采样: 累加 v² + 正向过零结算本周期 RMS + 状态机推进 (含 slew 毛刺过滤 + 短周期丢弃)
- `power_event_step(me, cycle_rms)` — 应用自行结算周期 RMS 后直接推进
- `power_event_active(me)` — 1=有事件进行中
- `power_event_reset(me)` — 重置状态与计数 (保留参数)

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_arc_detect.h — 光伏电弧检测 (FFT 频带能量 + 阈值)

> **来源:** TI C2000Ware Digital Power SDK solutions/tida_010231/source/arc/source/ArcDetect.c
> **新增日期:** 2026-08-12

直流电弧在频谱上呈现宽频带能量抬升 (宽带噪声), 而开关谐波集中在窄带 — 检测器据此区分: 分析频带 [bin_min, bin_max) 等分低频/高频两半, 各自求最小能量; 单频干扰滤除 (重复 `FilterBins = NumBins·D·0.5` 次, 把每半带最强 bin 替换为本半带最小值) 剔除开关谐波; 频带能量 `BandSum = F·Σ(低频带) + Σ(高频带)`; 转 dB `dB = 10·log10(BandSum) + 2.129 − 90.31 + AD_correction` (2.129 = Hanning 修正, 90.31 = 满刻度偏置); `dB > 阈值 T` 即判电弧. 与 FFT 后端解耦, 输入取幅值平方谱.

**关键 API (static inline):**
- `arc_detect_init(me, bin_min, bin_max, band_weight, bin_discard, threshold)` — F=低频带加权 (默认 64), D=滤除比例 (默认 0 关), T=判定阈值 (默认 220)
- `arc_detect_run(me, mag_sq)` — FFT 后传入幅值平方谱, 返回频带能量 (dB)
- `arc_detect_check(me)` — 读取电弧判定 (1=电弧)

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_pid_nl.h — 非线性 PID (各通路幂律整形)

> **来源:** TI C2000Ware Digital Power SDK libraries/control/DCL/c28/include/DCL_NLPID.h
> **新增日期:** 2026-08-12

并行式非线性 PID: P/I/D 三通路各自经整形函数 `f(e) = sign(e)·|e|^α` (|e|>δ) 或 `e·γ` (|e|≤δ). α<1 小误差放大 (响应快), α>1 小误差衰减 (抗噪), γ 为线性区增益 — 用 `nl_pid_gamma_from_delta(α,δ) = δ^(α−1)` 保证两区边界连续. 积分带抗饱和 (i16 标志), 微分带二阶滤波 (c1/c2). 输入/输出归一化 ±1 (pu), 误差在 run 内折半预处理 (与 DCL 一致). 用途: 电源启动、负载突变等强鲁棒场景.

**关键 API (static inline):**
- `nl_pid_cfg_default()` / `nl_pid_init(me)` — 默认配置 (线性等价 kp=1) + 状态清零
- `nl_pid_gamma_from_delta(α, δ)` — γ = δ^(α−1), 边界连续辅助
- `nl_pid_set_filter_bw(cfg, fc, dt)` — 微分滤波器带宽双线性换算 c1/c2
- `nl_pid_run(me, cfg, ref, fdb, clamp_flag)` — 单步, 返回限幅后输出

**依赖:** `<math.h>`

#### comp_tcm.h — 控制器自动调参 (触发式阶跃捕获 + 性能准则)

> **来源:** TI C2000Ware Digital Power SDK libraries/control/DCL/c28/include/DCL_TCM.h
> **新增日期:** 2026-08-12

TCM (Tuning Criteria Module): armed 状态持续把误差写入预触发环形缓冲; 误差越限 (e>trigMax 或 e<trigMin) 触发捕获 — 回填 lead 个预触发样本, 触发样本作为首个后触发样本, 继续捕获至窗口满 size 样本. 对捕获的误差响应求性能准则: IAE=Σ|e|, ISE=Σe², ITAE=Σ|e|·t. 自动调参: 对每个候选增益重放相同响应, 取准则最小者为最优.

**关键 API (static inline):**
- `tcm_init(me, buf, lead, size, trig_min, trig_max)` — 捕获缓冲由调用者提供 (长度 ≥ size)
- `tcm_arm(me)` / `tcm_disarm(me)` — 使能/取消捕获
- `tcm_run(me, e)` — 每采样周期传误差信号 (IDLE/COMPLETE 下为空操作)
- `tcm_iae(err, n)` / `tcm_ise(err, n)` / `tcm_itae(err, n, dt)` — 性能准则

**依赖:** `<stdint.h>`, `<math.h>`

#### comp_sgen.h — 信号发生器库 (7 种发生器)

> **来源:** TI controlSUITE SGEN (FPUfastRTS sincos) + DCL SGEN + dsp/SGEN/v101
> **v1.1 扩展:** HP1/HP2/T3D/Profile/Deadzone (5 种高级发生器)
> **新增日期:** 2026-08-12

**7 种发生器类型:**
| 发生器 | 结构体 | 用途 |
|--------|--------|------|
| `SgenFixed` | 定频 DDS | 正弦参考波 (幅值/频率/相位/DC偏移可调, 相位连续调频) |
| `SgenSweep` | 线性扫频 (Chirp) | 扫频阻抗测量 / Bode 图激励, 扫完静音 |
| `SgenHp1` | 单音硬件探测 | 系统辨识单频注入, 输出 sin_val/cos_val 正交分量供 IQ 解调 |
| `SgenHp2` | 双音硬件探测 | 互调失真 (IMD) 测试, 双频同时注入, 独立分量输出 |
| `SgenT3D` | 三音 DDS 合成 | 多频阻抗/谐波合成/电网模拟, 3 路独立频率/幅值/相位 |
| `SgenProfile` | 分段线性 Profile | 软启动/速度规划/任意波形, 最多 16 断点线性插值 |
| `SgenDeadzone` | 死区线测试 | PWM 死区补偿校准, 交替活动区/零区, 双极性/单极性可选 |

**关键 API (全部 static inline):**
- `sgen_fixed_init/tick/set_amplitude/set_phase/reset_phase(me)` — 定频 DDS 全功能
- `sgen_sweep_init/tick/reset(me)` — 线性 chirp 扫频
- `sgen_hp1_init/run/set_freq(me)` — 单音探测 (含正交分量)
- `sgen_hp2_init/run(me)` — 双音叠加 (独立 sin_val1/sin_val2)
- `sgen_t3d_init/set_tone/run(me)` — 三通道独立 DDS + 合成
- `sgen_profile_init/load/run/reset/is_done(me)` — 断点插值, 完成钳位
- `sgen_deadzone_init/run/reset(me)` — 活动/零区交替, 双极性反转

**依赖:** `<math.h>` (sinf/cosf)

#### comp_resolver.h — 旋转变压器接口 (浮点 + IQmath 定点)

> **来源:** TI controlSUITE motor_control/math_blocks/v4.3 (resolver.h) + motor_control/libs/resolver/v101 (Resolver_Fixed.h)
> **v1.1 扩展:** IQmath (Q24) 定点完整信号链 (DDS 励磁 + 同步解调 + LPF + atan2 查表 + PLL 锁相环)
> **新增日期:** 2026-08-12

**双路径:**
- **浮点路径 (Resolver)** — 简单解算: 机械角度 = (raw - offset) × 2π/steps_per_turn; 电角度 = pole_pairs × 机械角度
- **IQmath 定点路径 (ResolverFixedCfg + ResolverFixedState)** — 完整旋变信号链, 零浮点依赖, 适合无 FPU 平台

**IQmath 信号链 (5 步):**
1. DDS 励磁载波生成 → sin 参考输出 (DAC/PWM)
2. ADC 同步采样 sin/cos 调制信号
3. 同步解调: ADC × 励磁参考 → 提取包络
4. 一阶 IIR LPF 滤除 2ω 残差 → sin_dc/cos_dc
5. atan2 查表 + PLL 锁相环 → 滤波角度 + 速度估计

**关键 API (全部 static inline):**
- `resolver_init/decode/set_offset(me)` — 浮点简单解算
- `resolver_fixed_cfg_default(fs)` → 默认配置 (励磁 10kHz, LPF 500Hz)
- `resolver_fixed_excite(cfg, st)` — DDS 相位累加, 返回励磁 sin 参考 (Q24)
- `resolver_fixed_demodulate(cfg, st, sin_adc, cos_adc)` — 5 步解调 + PLL, 返回电角度 Q24 per-unit
- `resolver_fixed_get_angle_rad/get_speed(cfg, st)` — Q24→float 弧度 / rad/s

**依赖:** `<math.h>`, `<stdint.h>`, `comp_iqmath.h`

#### comp_fft_window.h — FFT 窗函数库 (18 种窗, float + Q31)

> **来源:** TI controlSUITE FPU library (18 独立头文件) + FixedPointLib (fft_*_Q31.h)
> **v1.2 扩展:** Q31 定点窗系数生成 + 加窗 (fill_q31 / apply_q31)
> **新增日期:** 2026-08-12

**18 种窗类型 (FftWinType 枚举):** RECTANGULAR / BARTLETT / HANN / HAMMING / BLACKMAN / BLACKMAN_HARRIS / BLACKMAN_NUTTALL / NUTTALL / FLATTOP / BOHMAN / GAUSS (α=2.5) / KAISER (β=6) / CHEBYSHEV (80dB) / TUKEY (α=0.5) / PARZEN / TAYLOR / TRIANGULAR / BARTHANN

**核心 API (全部 static inline):**
- `fft_win_generate(type, win, size)` — 生成 N 点 float 窗系数 (大部分已归一化)
- `fft_win_apply(win, data, dst, size)` — 逐点加窗 (支持就地, dst == data)
- `fft_win_coherent_gain/enbw(win, size)` — 相干增益/等效噪声带宽 (FFT 幅值/噪声校正)

**Q31 定点扩展:**
- `fft_win_fill_q31(type, buf, n)` — float 生成 → Q31 转换 (× 2^31-1, double 中间精度)
- `fft_win_apply_q31(win, data, n)` — Q31 就地加窗 (int64 乘累加 >>31, 防溢出)
- `fft_win_apply_q31_dst(win, data, dst, n)` — Q31 非就地加窗
- `fft_win_coherent_gain_q31(win, n)` — Q31 定点相干增益

**依赖:** `<math.h>`, `<stdint.h>`

#### comp_iir.h — IIR 数字补偿器库 (float + Q15/Q31 定点)

> **来源:** TI controlSUITE DCL (DF22/DF23) + Solar (CNTL_2P2Z_F) + FixedPointLib/v1_20 (iir.h, Q15/Q31)
> **v1.2 扩展:** Q15/Q31 定点 IIR (Iir16/Iir32, Direct Form 1, 最大 8 阶)
> **新增日期:** 2026-08-12

**三种 float 实现:**
| 变体 | 结构体 | 结构 | 特点 |
|------|--------|------|------|
| DF22 | `IirDf22` | Direct Form 2, 二阶 | CMSIS-DSP biquad 兼容, 最少状态存储 (2 延迟) |
| DF23 | `IirDf23` | Direct Form 2, 三阶 | LCL 滤波器谐振抑制等高阶补偿 (3 延迟) |
| 2P2Z | `Iir2p2z` | Direct Form 1, 二阶 | 经典控制环补偿, 两级饱和: 软限制 (i_min ≥ -0.9) → 硬限制 (min) |

**关键 API (全部 static inline):**
- `iir_df22_init/run/reset(me)` — 二阶 DF2: v = in - a1·x1 - a2·x2
- `iir_df23_init/run/reset(me)` — 三阶 DF2
- `iir_2p2z_init/run/reset(me)` — 2P2Z 两级饱和: 软限制保护内部状态 → 硬限制输出

**Q15/Q31 定点扩展 (v1.2):**
- `Iir16Cfg` / `Iir16State` — Q15 Direct Form 1, 最大 8 阶, int64 累加防溢, 输出饱和 [-32768, 32767]
- `Iir32Cfg` / `Iir32State` — Q31 Direct Form 1, 最大 8 阶, 高精度变体
- `iir16_init/run(cfg)` / `iir32_init/run(cfg)` — 延迟线移位 + 乘累加 + 后缩放右移归一化

**依赖:** `<stdint.h>`

### 8.9 Module — 频响分析仪 (SFRA)

> **来源:** TI controlSUITE SFRA/v1.20/Float, 翻译为 HardC 纯C float 版本
> **新增日期:** 2026-08-12

**SFRA 软件频率响应分析仪 — 在线 Bode 图测量，无需外接 FRA 仪器。**

**架构：**
```
SfraCfg (用户配置: f_start/end, inject_amp, points_per_decade, settle/measure cycles)
  └── Sfra (运行时状态: DDS 正弦注入 + DFT 累加器 + 增益/相位结果)
```

**四阶段扫频流程：**
1. **DDS 正弦注入** — 在控制环参考点叠加小信号扰动
2. **DFT 采集** — 注入点和响应点做激励频率的 Goertzel DFT
3. **增益/相位** — `20×log10(|H|)`, `∠H = ∠resp - ∠inj`
4. **对数扫频** — `freq[k] = f_start × 10^(k × decade_step)`

**典型 ISR 调用链：**
```c
mod_sfra_inject(&sfra);
ref += sfra.inject_out;         // 扰动叠加到参考点
/* ... 控制算法 ... */
mod_sfra_collect(&sfra, output);
```

**主循环回调：** `on_point_done` 回调每频点完成后触发 (用于串口/GUI 发送结果), 支持用户自定义数据上下文。

**依赖：** `<math.h>` (sinf/cosf/sqrtf/atan2f/log10f/powf)

### 8.10 Module — 快速电流环 (FCL)

> **来源:** TI controlSUITE motor_control/libs/FCL, 翻译为 HardC Module 层纯C float 版本
> **新增日期:** 2026-08-12

**FCL 快速电流环 — dq 旋转坐标系高带宽 PI 电流控制 + 交叉解耦 + 反电动势前馈 + 有源阻尼。**

**架构：**
```
FclCfg (用户配置 POD, YAML 注入目标: dt/ld/lq/rs/flux_pm/v_dc_max/i_max/Kp/Ki)
  └── FclCtrl (运行时 Instance: PI 积分器 + 输出电压 + 故障检测)
```

**运行模式 (FclMode)：**
- `Idle` — 空闲, PWM=0
- `Enabled` — 正常电流控制
- `Fault` — 故障锁定, 需手动 `fcl_clear_fault()`

**ISR 热路径 (每 PWM 周期)：**
```c
// Park 变换后得到 i_d, i_q
fcl_run(&fcl, i_d, i_q, i_d_ref, i_q_ref, omega_e, v_bus);
// 用 fcl.v_d_ref, fcl.v_q_ref 做反 Park → αβ → SVPWM
```

**控制算法 (在 `fcl_run` 中)：**
1. **d 轴 PI** — 磁链电流控制 (id_ref 通常=0 for MTPA 以下)
2. **q 轴 PI** — 转矩电流控制 (iq_ref 来自速度环或转矩指令)
3. **交叉解耦** — `v_d += -ω·Lq·iq`, `v_q += +ω·(Ld·id + ψm)` — 消除 dq 轴耦合
4. **反电动势前馈** — `v_q += ω·ψm` — 减少 PI 积分负担
5. **有源阻尼** — `v_d += kp_damp·(id_ref - id_meas)` — 抑制 LC 谐振 (可选)
6. **电压限幅** — `sqrt(v_d² + v_q²) ≤ v_dc_max / √3` (SVPWM 线性调制区)

**主循环慢速路径：** `fcl_get_fault(me)` → 检查故障码 → `fcl_clear_fault(me)` 清除锁存

**故障检测：** 过流去抖计数 (`overcurrent_cnt`), fault_code bitmask

**关键约束：**
- `fcl_run` 禁止 printf / 浮点除零 / 阻塞等待
- PI 积分器有独立抗饱和 (v_d/v_q 限幅后反算积分限)
- dt/L/R/ψ 等电机参数必须在上电时从 YAML 注入, 运行中通过 apply_config 同步

**依赖：** `<stdbool.h>`, `<stdint.h>`, `<math.h>`

### 8.11 Module — PMBus 协议栈

> **来源:** TI controlSUITE comms/PMBus, 翻译为 HardC 纯C 版本
> **新增日期:** 2026-08-12

**PMBus 协议栈 — 基于 I2C 从机的数字电源通信协议 (SMBus 2.0 物理层 + PMBus 1.3 命令层)。**

**架构：**
```
PmbusCmdEntry[] (用户命令表: 命令码 → 读/写回调)
  └── Pmbus (运行时 Instance: I2C 缓冲 + 命令解析 + 状态寄存器)
```

**数据格式：**
| 格式 | 编码 | 典型用途 |
|------|------|---------|
| Linear11 | V = X · 2^N, X=11-bit signed, N=5-bit signed | VOUT/IOUT/TEMP 通用 |
| Linear16 | 同上, X=16-bit | 高精度读数 |
| Direct | X = (1/m) · (Y·10^(-R) - b) | 制造商自定义 |

**标准命令集 (子集, PmbusCmd 枚举, 可扩展)：**
- 控制: `OPERATION` / `ON_OFF_CONFIG` / `WRITE_PROTECT` / `CLEAR_FAULTS`
- 输出电压: `VOUT_MODE/COMMAND/MAX/TRANSITION/DROOP/SCALE_LOOP/SCALE_MONITOR`
- 监测: `READ_VIN/VOUT/IOUT/TEMP_1/TEMP_2/DUTY/FREQ/POUT/PIN`
- 状态: `STATUS_BYTE/WORD/VOUT/IOUT/INPUT/TEMP/CML`
- 制造商: `MFR_ID/MODEL/REVISION` + `MFR_SPECIFIC_BASE` (0xD0~0xFF)

**关键 API：**
```
pmbus_init(me, cmd_table, cmd_count, cmd_ctx)  →  绑定命令表和用户上下文
pmbus_on_rx(me)                                 →  I2C RX ISR 中调用, 解析命令→查表→调回调
pmbus_on_tx_byte(me)                            →  I2C TX 请求, 返回下一待发送字节 (-1=完成)
pmbus_linear11_encode/decode(value, exponent)    →  Linear11 格式转换
pmbus_get_status_byte/word(me)                  →  读取 PMBus 标准状态
pmbus_set_status_bit(me, bit, active)           →  设置状态位 (同步更新 byte+word)
```

**命令表条目 (PmbusCmdEntry)：**
- `cmd` — 命令码 (PmbusCmd 枚举)
- `name` — 调试用名称
- `writable` — 是否可写
- `data_len` — 数据长度 (字节)
- `on_read(ctx, data, max_len)` → 返回实际长度 (NULL=不支持读)
- `on_write(ctx, data, len)` → 返回处理结果 (NULL=不支持写)

**典型用法：**
```c
// 1. 定义命令表
static const PmbusCmdEntry pmbus_cmds[] = {
  { PMB_CMD_READ_VOUT, "VOUT", false, 2, pmbus_read_vout, NULL },
  { PMB_CMD_VOUT_COMMAND, "Vset", true, 2, pmbus_read_vset, pmbus_write_vset },
  // ...
};

// 2. 初始化
pmbus_init(&pmbus, pmbus_cmds, ARRAY_SIZE(pmbus_cmds), &power_ctrl);

// 3. I2C RX ISR → pmbus_on_rx(&pmbus);
// 4. I2C TX 请求 → int byte = pmbus_on_tx_byte(&pmbus);
```

**依赖：** `<stdint.h>`, `<stdbool.h>`

### 8.12 VCU 信道编码组件

> **来源:** TI controlSUITE VCU 库 (v2_00/v2_10), 翻译为 HardC 纯C inline 版本
> **新增日期:** 2026-08-12

**应用场景:** 数字电视 (DVB-C/S/T) / 电力线通信 (PLC/G3-PLC) / 无线充电通信 (Qi/NFC) / 深空通信级联码

#### comp_rs.h — Reed-Solomon RS(255,239) 编解码器

> **来源:** TI controlSUITE VCU/v2_00/reedsolomon_encoder.h + vcu2_reedsolomon_decoder.h

**RS(N=255, K=255-2T, T=1..16):** 系统码 (消息在前, 校验在后), GF(2^8) 本原多项式 0x11D = x^8+x^4+x^3+x^2+1, exp/log 查表 (512/256 条目免取模), 可纠正 T 字节错误

**GF(256) 基本运算 (全部 static inline):**
- `gf256_mul(a, b)` — log-exp 查表 O(1), 零元检测直通
- `gf256_inv(a)` — a^(-1) = α^(255 - log_α(a))
- `gf256_div(a, b)` — a × b^(-1)

**核心结构体:**
- `RsCfg` — 配置 (error_cap/block_len/msg_len/gen_poly[2T+1]), `rs_init(cfg, T)` 构建生成多项式 g(x) = Π(x - α^i)
- `RsState` — 解码器运行状态 (syndromes/lambda/omega/err_pos/err_mag/nerr)

**编码:** `rs_encode(cfg, msg[K], codeword[N])` — LFSR 多项式除法, 消息原样复制 + 2T 字节校验追加

**解码流水线 (rs_decode, 5 步):**
1. **伴随式计算** — `rs_calc_syndromes(st, cfg, rx)` — Horner 算法求 S_i = r(α^i), 返回 0=无错
2. **Berlekamp-Massey** — `rs_berlekamp_massey(st, cfg)` — 迭代求 Λ(x): 偏差 δ≠0 时 Λ += δ·B(x), 2L≤r 时更新 B(x)
3. **Chien 搜索** — `rs_chien_search(st, cfg, deg_lambda)` — 增量求值 Λ(α^(-j)), 根数须 = 度数, 否则不可纠正
4. **Forney 算法** — `rs_forney(st, cfg, deg_lambda, nerr)` — Ω(x) = S(x)·Λ(x) mod x^(2T), 错误幅度 = X·Ω(X^(-1)) / Λ'(X^(-1))
5. **纠错** — corrected[pos] = received[pos] ^ err_mag[pos]

**解码器返回:** ≥ 0 = 纠正数, -1 = 不可纠正

**依赖:** `<stdint.h>`, `<string.h>` (memset/memcpy)

#### comp_interleaver.h — 卷积交织器/解交织器

> **来源:** TI controlSUITE VCU/v2_10/common/interleaver.h + vcu2_deinterleaver.h

**Forney/Ramsey 型卷积交织:** B 条分支 (0..B-1), 分支 i 延迟 = i×D 符号 (交织) / (B-1-i)×D 符号 (解交织, 互补抵消), 总缓冲 = D×B×(B-1)/2 符号, 端到端延迟 = (B-1)×D 符号 (恒定)

**核心作用:** 将信道突发错误分散到多个 RS 码字, 化突发为随机, 配合 RS 编解码实现级联纠错

**核心结构体:**
- `InterleaverCfg` — 配置参数 (branches ∈ [2,128], delay ∈ [1,32], sym_size ∈ [1,4])
- `Interleaver` — 运行时状态: 线性环形缓冲 + 分支基地址预计算 (branch_base[128]) + 写指针 (wptr[128]) + is_deinterleaver 标志区分模式

**关键 API (全部 static inline):**
- `interleaver_cfg_default()` — 默认: B=16 分支, D=4
- `interleaver_get_buffer_size/syms(cfg)` — 计算所需缓冲区容量
- `interleaver_init(me, cfg, buf, size)` — 交织器初始化, 预计算分支基地址 + 清零缓冲
- `deinterleaver_init(me, cfg, buf, size)` — 解交织器初始化, 延迟模式镜像互补
- `interleaver_write/deinterleaver_write(me, sym_in, sym_out)` — 先读后写 FIFO: 零延迟分支直通, 非零延迟先取旧值再写新值, 写指针环形回绕
- `interleaver_flush/deinterleaver_flush(me, out, max)` — 块结束写入零符号, 推出残余数据
- `interleaver_reset(me)` — 清空缓冲, 复位所有指针

**典型配置:** DVB 标准: B=12, D=17 (配合 RS(204,188))

**依赖:** `<stdint.h>`

#### comp_checksum.h — 校验和计算 (8/16/32 位累加)

> **来源:** 原 Components/math/comp_math.h 拆分 (2026-08-14)

**无符号累加校验和:** 逐元素相加, 自然溢出截断, O(n), 无查表/无依赖, ISR 安全, 与 comp_crc.h (多项式校验) 互补

**关键 API (全部 static inline):**
- `math_sum_u8(addr, len)` — 逐字节累加, 返回 uint8_t
- `math_sum_u16(addr, len)` — 逐 16 位字累加, 返回 uint16_t
- `math_sum_u32(addr, len)` — 逐 32 位字累加, 返回 uint32_t

**依赖:** `<stdint.h>`

#### comp_endian.h — 大小端转换 (16/32 位)

> **来源:** 原 Components/math/comp_math.h 拆分 (2026-08-14)

**字节序翻转:** 原地修改或源→目标双缓冲拷贝, 用于外部大端协议 (PMBus/SMBus) 与内部小端数据交换

**关键 API (全部 static inline):**
- `math_endian_reverse_16(addr)` — 16 位原地翻转 (字节 0↔1)
- `math_endian_reverse_16_copy(src, dst)` — 16 位翻转后拷贝
- `math_endian_reverse_32(addr)` — 32 位原地翻转 (字节 0↔3, 1↔2)
- `math_endian_reverse_32_copy(src, dst)` — 32 位翻转后拷贝

**依赖:** `<stdint.h>`

#### comp_ask.h — ASK/OOK 无线充电信令 codec

> **来源:** 无线充电 (WPT) E 侧 2000Hz ASK 包络调制 (2026-08-14)

**包格式 (12 bit, MSB-first, uint16_t 低 12 位):** bit11=req (1=请求充电), bit10..3=power_w (0..255), bit2=EVEN 偶校验 (覆盖 bit11..3), bit1..0=pad (恒 0)

**奇偶校验:** 全 12 位 XOR 归约 = 0 → `ask_parity_ok` true; 任何单比特翻转破坏偶性被检出

**关键 API (全部 static inline):**
- `ask_encode_pkt(req, power_w)` — 编码 12-bit 包 (偶校验位自动置位)
- `ask_parity_ok(pkt)` — 偶校验通过检测
- `ask_decode_pkt(pkt, &req, &pw)` — 提取 req + power_w (指针可 NULL)
- `AskDecode_Init/Update(me, carrier)` — 2000Hz 包络采样解码状态机 (20 采样/位多数表决, 首载波采样触发同步, 收满 12 位自动重同步支持连续流, 返回 true=收满一包)

**时序常量:** `ASK_CARRIER_HZ=2000 / ASK_BIT_RATE_HZ=100 / ASK_PKT_BITS=12 / ASK_SAMPLES_PER_BIT=20`

**依赖:** `<stdint.h>`, `<stdbool.h>`

### 8.13 Module — 超级电容功率管理 (SuperCap)

> **来源:** WEILAI 三相并联超级电容 + HKUST 2024 F3 / 2025 PCM 历史版 (2026-08-14)
> 对照 [docs/learning/SuperCap_Projects_Study_Report.md](docs/learning/SuperCap_Projects_Study_Report.md). 拓扑 `supercap_3ph.yaml`, `phases: 1..3`.

**三相并联超电核心差距:** 单级 → N 相并联 (相序映射 + 均流 + 切换同步 + N×电流 PI). 相位参数化 N=1..3, 单相时均流/相序/切换同步退化为无操作.

#### mod_supercap.h/c — 功率环 (PowerStage 派生, ctx fast)

**级联 (每 28.3kHz tick):**
```
p_referee = LPF(power_lpf, va × i_chassis)   # 实测裁判功率 (120Hz)
p_setpoint = PI(pid_p, sp=referee_power, fbk=p_referee)   # 期望功率
p_setpoint clamp [p_lim_lo, p_lim_hi]        # 功率限幅 (i_lim×va / cap_ilimit×vb)
taper 近满压 → i_side = p_setpoint / va
```

**保护:** charge_ok Hysteresis (28.6/28.0) 满电停充, discharge_ok Hysteresis (18/19) 低压切除, short_deb/unbalance_deb Debounce → FAULT (0x08/0x40) → 急停 + 事件环

**Cfg POD 槽位 0-9** (supercap_3ph.yaml params.slot 一致): pid_p_kp/ki, charge_stop_v(28.6), charge_resume_v(28.0), vcut_lo(18), vcut_hi(19), i_lim_a, cap_in_ilimit, cap_out_ilimit, share_gain

**PowerStage 基类重解释 (头注释声明):** base.vref=充电目标电压, base.iref=裁判电流参考 (派生值, 非控制输入); 级联绕过 base.loop[2] (per-phase PI 归 mod_current_share)

**关键 API:** mod_supercap_init/bind/sync_cfg/tick/start/stop/emergency/apply_tune; MAIN 侧 mod_supercap_set_referee_power (命令邮箱周期边界生效), mod_supercap_evt_pop (事件环排空)

**依赖:** comp_power_stage/pi_reg4/filter/protection/math/error/ring/latch/mailbox + adc_dc_sampler + mod_current_share

#### mod_current_share.h/c — 三相均流 (普通结构, ctx fast)

**职责 (每 tick):** ratio=vb/va → 模式迟滞 (0.97/1.03 进, 0.90/1.10 出) → ibase=paside/N → 每相 share_error=clamp(share_gain×(iavg−iphase)) → 电流 PI (sp=ibase+share_error, fbk=iphase) → alpha=1+u → ModeSync 切换同步 → 写 PwmBuckBoost

**N=1 退化:** iavg=iphase → share_error=0, ibase=paside, ModeSync(1)=恒等; 迟滞仍选模式

**关键 API:** mod_share_init/bind/tick/emergency/release; FAST 注入 set_voltages/set_paside/set_currents

**依赖:** comp_pi_reg4/protection/math + pwm_buckboost

#### mod_can_proto.h/c — 精简 CAN 协议 (ctx main, 新文件)

**帧:** 0x051 遥测 (200Hz: refereePowerLimit u8 / chassisPower/refereePower/SuperCapOutputMx u16 编码 / OutPutCapability u8); 0x061 接收 (enableCONV bit + refereePowerLimit u16)

**功率编码 (WEILAI mod_conn.h):** u16 = p×64+16384, 量程 -256W~+768W, 分辨率 0.015625W

**关键 API:** mod_can_init/bind/tx_telemetry/poll/on_frame; I/O 接缝回调 (send/poll/on_referee) 可绑任意 CAN 设备, host 可测

**ctx:** main — 收发全部主循环/低速上下文, 不进 28kHz 控制 ISR


