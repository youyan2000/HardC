# HardC —— 嵌入式 C 面向对象的电力电子/电机控制硬件驱动框架

> 纯 C99 OOP | 五层扁平架构 | 多平台 (STM32 + C2000) | 文件前缀区分域
> 仓库：`https://github.com/youyan2000/HardC` ｜ 装配线工具链见 [yamc](../yamc/README.md)（本地）｜ 完整文档在根容器 [../docs](../docs)（本地）

---

## 定位声明（最先理解，违反即误删库资产）

**HardC 是组件库，不是可执行工程。**
- 未调用的组件/函数是**库存**（供未来任意拓扑按需选用），**不是死代码**。库的价值 = **可复用**，不是"被当前 App 调用"。一个拓扑只用 5 个子系统，其余 40+ 组件是库存。
- **硬规则：禁止以"无调用者 / 未被当前拓扑使用"为由删除、废弃或标注任何库代码。** 合理的目录/改名重整（`git mv` 保留内容）不算删除，但内容必须保留。
- 具体拓扑（buck / supercap_3ph / vsi_3ph …）只是从库中选配的**消费示例**，**不限定库的范围**。库服务于无数拓扑，拓扑不反过来裁剪库。
- 相关教训：#56（库 vs 程序）、#45（误删已提交算法）。Reviewer 检查清单含「库资产保护」一节（code-review-workflow SKILL）。

## 架构

```
BSP/ → Components/ → Devices/ → Module/ → App/
```

| 层 | 目录 | 命名前缀 | 角色 |
|:---|:---|:---|:---|
| L1 BSP | `BSP/` | `bsp_*` | 平台抽象：不透明句柄，隔离 STM32 ↔ C2000 |
| L2 Components | `Components/` | `comp_*` | 纯算法父类 + ops 虚表，不看寄存器 |
| L3 Devices | `Devices/` | `<域>_<子类>` | 具体硬件子类，板级引脚绑定 |
| L4 Module | `Module/` | `mod_*` | 业务模块，只通过 Base* 句柄操作 |
| L5 App | `App/` | `app_main.*` | 应用入口：根结构体、ISR、配置同步 |

> 45 Components, 35+ Devices, 13 Modules, 8 BSP 文件；每层子目录含 `MANIFEST.yaml` 自描述与依赖声明。

## 目录结构

```
hardc/
├── BSP/              # L1 平台抽象（bsp_hrtim/adc_stm32/gpio_c2000…）
├── Components/       # L2 父类 + 纯算法（adc/ pid/ pwm/ pll/ motor/ math/ codec/ contract/…）
├── Devices/          # L3 具体子类（adc_*/ pid_*/ pwm_*/ com_*/ per_*…）
├── Module/           # L4 业务模块（motor/ power/ comm/ hmi/ bootloader）
├── App/              # L5 模板（app_main.c/h.tmpl, bootloader_main.tmpl, pid_tune）
├── cmake/            # 工具链文件（starm-clang/gcc-arm-none-eabi/c2000-ti-cgt + HardC.CMake）
├── Config/           # 拓扑定义 topologies/ + 分区 flash_map.yaml + 示例 project/params
└── .github/          # CI（格式检查 + 编译拓扑）
```

## 核心文档 / 导航（字典）

| 文档 | 内容 |
|------|------|
| [../docs/concept.md](../docs/concept.md) | **设计原则（唯一思想权威）**：定位、三上下文、五层、五原语、复用 libxr 边界 |
| [../docs/PLAN.md](../docs/PLAN.md) | **唯一主线计划** — 已完成归档 + 未完成待办 |
| [../docs/coding/oop.md](../docs/coding/oop.md) | OOP 方法论：分层架构、OOP 核心模式(虚表/container_of)、风格、复用 |
| [../docs/subsystems/index.md](../docs/subsystems/index.md) | 子系统总览 + 按域导航（ADC/PID/PLL/PWM/Motor/… 各一文件） |
| [../docs/coding/protection-hmi.md](../docs/coding/protection-hmi.md) | FAST>SLOW>HMI 三档 + HMI 线程设计 |
| [../docs/history/lessons.md](../docs/history/lessons.md) | 调参教训库（78 条 + 经验模板），git 版本管理，禁止回退 |
| [../docs/guides/external-project-integration.md](../docs/guides/external-project-integration.md) | 外部 CubeMX 工程接入入门 |

> 本仓库为库本体；文档统一在本地根容器 `../docs/`（GitHub 上 HardC 不含 docs——按结构约定）。

## App 模板

| 文件 | 用途 |
|------|------|
| [App/app_main.h.tmpl](App/app_main.h.tmpl) | App 层头文件模板 — 根结构体、配置 POD、ISR 钩子声明 |
| [App/app_main.c.tmpl](App/app_main.c.tmpl) | App 层实现模板 — board_init、yamc 注入点、ISR 实现、BackgroundTask |
| [App/pid_tune.h/c](App/pid_tune.h) | PID 串口调参协议 (0xFB 帧, 48 字节) |

## AI 技能文件

| 技能 | 用途 |
|------|------|
| [.claude/skills/embedded-oop-c](../.claude/skills/embedded-oop-c/SKILL.md) | C 语言 OOP 完整方法论 |
| [.claude/skills/c-code-style](../.claude/skills/c-code-style/SKILL.md) | 代码风格：2空格缩进、K&R大括号、命名约定 |
| [.claude/skills/stm32-hal](../.claude/skills/stm32-hal/SKILL.md) | STM32 HAL/LL 专业知识 |
| [.claude/skills/code-review-workflow](../.claude/skills/code-review-workflow/SKILL.md) | **强制**写-审双 Agent 工作流 |
| [.claude/skills/project-orchestration](../.claude/skills/project-orchestration/SKILL.md) | 项目统筹：多 Agent 并发协作治理（plan 审批/角色边界/提交闸门） |
| [.claude/CLAUDE.md](../.claude/CLAUDE.md) | AI 协作入口（定位 + 行为准则 + 三仓库结构） |

---

## 中断优先级三档（库级强制约定）

> **严格三档：FAST > SLOW > HMI，由 `bsp_irq_apply()` 默认强制（失败停机），不靠工程自觉。**
> 详见 [../docs/coding/protection-hmi.md](../docs/coding/protection-hmi.md) 与 [BSP/bsp_irq.h](BSP/bsp_irq.h)。

1. **FAST**（控制定时器 ISR，抢优先 **0**）——采样→控制→发波→快保护。
2. **SLOW**（监控定时器 ISR，抢优先 **1**）——慢保护/心跳/喂狗。
3. **CTX_HMI**（HMI/通信中断，`App_OnHmiTick`，抢优先 **2**）——按键去抖/命令分发/收包入队。
4. 后台（BackgroundTask）主循环——慢 I/O（printf/OLED/协议/Flash）。
5. `bsp_irq_apply` 对**每一档列出的所有中断**强制写 0/1/2 并逐个读回校验；**返回 false = 误配，必须停机**，不允许带病运行。
6. 工程必须定义 `FAST_CTRL_IRQN` / `SLOW_CTRL_IRQN` / `HMI_IRQN` 三宏（STM32 IRQn / C2000 PIE 组占位）。**HMI 可同挂多个通信/按键源**（每个 UART/CAN/FDCAN/EXTI 是独立 IRQn），用 `HMI_IRQN_2..4` 补充，yamc 从 `.ioc` 探测全部并注入。
7. **禁止**把任何通信/HMI 中断优先级设到与 FAST 同级（0）——三档必须严格分离。

## App 层开发规则

> **App 层只有一组文件。禁止自由发挥。** 所有项目基于 [App/app_main.c.tmpl](App/app_main.c.tmpl) / [App/app_main.h.tmpl](App/app_main.h.tmpl)。

1. **根结构体 `ProjectRoot`** — 嵌入所有 Device + Module 实例（值包含，零 malloc）。
2. **配置 POD `ProjectConfig`** — 纯数据结构，yamc 注入目标，与运行时 Instance 分离。
3. **`board_init()`** — 唯一创建和绑定实例的地方：Device init → Module init → 指针注入 → ISR 启动。
4. **`apply_config()`** — 配置 POD → 运行时 Instance 同步（启动 + YAML 注入后 + 0xFB 调参后）。
5. **`App_OnControlTick()`** — ISR 调用，顺序: 传感器→HMI→控制算法→执行器。禁止 printf。
6. **`BackgroundTask()`** — 主循环，做所有耗时 I/O: printf、软件 I2C、OLED、串口应答。
7. **`extern ProjectRoot g_root`** — Module 可通过 extern 访问兄弟模块。

## Git 管理约定

### 分支
- `main` — 稳定分支，始终可构建、文档完整；功能修复在 `feature/<name>` / `fix/<name>` 合并回 main。

### 提交规范
- **每个子系统独立提交**（ADC 和 PWM 分开 commit）。格式：`<子系统>: <动词短语>`（如 `PWM: 修复 Interleaved 相位计算溢出`）。
- 英文为主（面向国际协作），中文附注可用。

### .gitignore 策略
- 忽略：`*.o *.elf *.bin *.hex *.map`、`build/ cmake-build-*/`、`.vscode/ .idea/`、`.DS_Store Thumbs.db`、`__pycache__/ *.pyc`、`.claude/settings.json`（本地权限配置）、`docs/AGENT-SYNC.md` 等内部规划、`docs/learning/` 学习报告。
- **不忽略**：`*.yaml`（Config/ 配置是源码）。

### AI 助手操作 Git 的规则
1. **不自动提交**（用户允许后才 commit）。
2. **不 force push** 到 main。
3. 提交前：跨子系统拆 commit；未跟踪构建产物先更新 .gitignore；文档一致性（api 变了同步 docs）。
4. 永不提交：API 密钥/token、编译产物、IDE 个人配置。
5. `git status` 先看再动手；`git diff --stat` 搞清影响范围。

### AI 助手代码生成规则（写-审双 Agent，强制）
1. Writer Agent 读需求 → 读现有代码 → 生成/修改。
2. Reviewer Agent 独立检查 → 对照检查清单 → 通过/不通过。
3. **绝不跳过 Reviewer**（纯 .md / YAML 例外，commit 注明 `no-review: <原因>`）。
4. Reviewer 必查：2空格缩进（禁4空格）、K&R大括号、container_of、ops 绑定、include guard、行尾无空白、无 tab。

### 代码风格自动化
```bash
find . -name '*.h' -o -name '*.c' | xargs clang-format -i
clang-tidy Components/pid/comp_pid.h -- -I BSP -I Components -I Devices
```

### 子系统独立性
文件按子系统归入 `Components/<域>/` + `Devices/<域>/` + `Module/<域>/` 子目录（含 `MANIFEST.yaml` 依赖声明）。改 API/增子类/改 ops 虚表签名 → **必须同步更新文档** + 检查 MANIFEST 依赖。

---

## 接入你的工程（用 yamc，别手工拼）

由装配线工具 [yamc](../yamc/README.md) 一键完成：
```bash
python ../yamc/yamc_cfg.py -d <工程根> --topology buck --git-source https://github.com/youyan2000/HardC.git
```
- HardC 以 **git submodule** 或 `xcopy` 拷贝进 `Middlewares/Third_Party/HardC`（**禁 mklink 同名 junction**，曾致仓库"全丢"，见 lessons #74）。
- yamc 定位**库根**优先级：`HARDC_LIB_DIR` env → `--hardc-path` → 同级 `../hardc` → 工程内 submodule。
- yamc 注入 `cmake/HardC.CMake`（`HARDC_DIR`/`HARDC_DRIVER`/include）+ 三宏 + 编译。

## 构建（库自身 / 独立构建）

```bash
python ../yamc/scaffold.py gen Config/projects/<topo>.yaml
cmake -S build/gen/<topo> -B build/out/<topo> \
  -DCMAKE_TOOLCHAIN_FILE=cmake/starm-clang.cmake   # ARM Clang（或 gcc-arm-none-eabi）
cmake --build build/out/<topo> -j$(nproc)
```

## BSP 硬件加速抽象层

> **Components 层禁止直接 include 平台加速库。** 硬件加速 (DSP/PWM/ADC) 经 `BSP/` 不透明句柄分发（lessons #17）。

| 文件 | 内容 |
|------|------|
| [BSP/bsp_dsp.h](BSP/bsp_dsp.h) | 硬件加速抽象 — sqrt/biquad, 平台检测 (CMSIS-DSP/C2000Ware/纯C回退) |
| [BSP/bsp_dsp_fir.h](BSP/bsp_dsp_fir.h) | FIR 滤波器硬件加速 (4 后端) |
| [BSP/bsp_dsp_fft.h](BSP/bsp_dsp_fft.h) | FFT 快速傅里叶变换 (RFFT + CFFT, 3 后端) |
| [BSP/bsp_pwm.h](BSP/bsp_pwm.h) | PWM 不透明句柄 + 物理参数 API |
| [BSP/bsp_adc.h](BSP/bsp_adc.h) | ADC 校准/启动抽象 |

## 复用方式

1. **直接拷贝需要的子系统目录**（`Components/<域>/` + `Devices/<域>/` + `Module/<域>/`，连同 `MANIFEST.yaml`），或用 `scaffold.py gen`。
2. **基于父类继承新子类** — 父类为第一成员、container_of 下溯、构造器绑定 ops。

## 子系统速查

> 每个域一个子目录：`Components/<域>/`（父类）+ `Devices/<域>/`（子类）+ `Module/<域>/`（业务逻辑）。详见 [../docs/subsystems](../docs/subsystems)。

| 域 | Component | Devices 子类数 | 句柄头文件 |
|----|-----------|--------------|-----------|
| ADC | `comp_adc.h/c` | 3 (Follower/DC/AC Sampler) | `adcs.h` |
| COM | `comp_comm.h/c` + `comp_dma_rx/tx.h` | 5 (Uart/Spi/I2c/Can/Gpio) + 协议模块 | — |
| Peripheral | `comp_output.h/c` + `comp_sensor.h/c` + `comp_mpu.h` | 8 (Led/Beep/Fan/Oled/Mpu6050/…) | `pers.h` |
| PID | `comp_pid.h/c` + `comp_tcm.h` | 14 子类 (Standard/Parallel/DCL/Pi/Reg4/Reg3/PR/QPR/P2PD/P/I/PD/NL/Cascade) | `pids.h` |
| PLL | `comp_pll_base.h/c` | 5 (Sogi/Srf/Notch/Ddsrf/SogiFll) | `plls.h` |
| PWM | `comp_pwm.h/c` + `comp_sgen.h` | 9 (BuckBoost/HalfBridge/FullBridge/Interleaved/Resonant/SEPIC/SPWM/SVPWM/WPT) | `pwms.h` |
| Motor | `comp_motor.h/c` + instaspin + mod6 + 无感观测器家族 | 2 (TIM + Encoder) | — |
| StepMotor | `comp_step_motor.h/c` | 1 | — |
| Codec | crc/checksum/endian/viterbi/interleaver/rs/ask | 7 | — |
| Contract | `comp_io.h` + double_buffer/latch/ring/mailbox | — | — |
| Math | `comp_math.h` + iqmath + error + complex | — | — |

**独立组件（单头 static inline）**：comp_dlog / comp_vector / comp_pfc / comp_power_meas / comp_power_goertzel / comp_power_event / comp_power_fund / comp_protection / comp_protection_3lvl / comp_database / comp_bldc_instaspin / comp_impulse / comp_mod6 … 详见 [../docs/subsystems](../docs/subsystems) 与各 MANIFEST。

**Module 层（L4 业务逻辑）**：MotApp / TurnCtrl / Follower / HMI / ModBalance / CmdDispatch / ModComm / SerialProto / SFRA / FCL / PMBus / SuperCap / CurrentShare / CanProto …