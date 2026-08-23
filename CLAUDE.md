# CLAUDE.md — HardC 项目入口（AI + 人类共读）

> 本文件是 AI 助手和人类协作者的共同入口。它记录项目级别的约定，不重复代码本身已有的信息。

## 项目概述

**定位声明（最先理解，违反即误删库资产）：HardC 是组件库，不是可执行工程。**
- 未调用的组件/函数是**库存**（供未来任意拓扑按需选用），**不是死代码**。库的价值 = **可复用**，不是"被当前 App 调用"。一个拓扑只用 5 个子系统，其余 40+ 组件是库存。
- **硬规则：禁止以"无调用者 / 未被当前拓扑使用"为由删除、废弃或标注任何库代码。** 合理的目录/改名重整（`git mv` 保留内容）不算删除，但内容必须保留。
- 具体拓扑（buck / supercap_3ph / vsi_3ph …）只是从库中选配的**消费示例**，**不限定库的范围**。库服务于无数拓扑，拓扑不反过来裁剪库。
- 相关教训：#56（库 vs 程序）、#45（误删已提交算法）。Reviewer 检查清单含「库资产保护」一节（code-review-workflow SKILL）。

纯 C 语言实现的面向对象嵌入式硬件驱动库。5 层目录：BSP → Components → Devices → Module → App，**文件按子系统归入子目录隔离**，每层子目录含 `MANIFEST.yaml` 自描述。跨平台：STM32 (HAL/HRTIM) + TI C2000 (ePWM/CLA) + 纯C。

**目录结构：**
```
HardC/
├── BSP/             # L1: 不透明句柄 → 平台抽象
├── Components/      # L2: comp_*.h/c → 父类 + ops 虚表
│   ├── adc/ comm/ pid/ pwm/                # 父类域
│   ├── peripheral/ dsp/ motor/ power/ protection/ math/ codec/ contract/ database/
│   └── (每子目录含 MANIFEST.yaml 自描述)
├── Devices/         # L3: <域>_<子类>.h/c → 具体硬件实现
│   ├── adc/ comm/ pid/ pwm/
│   └── peripheral/ motor/
├── Module/          # L4: mod_*.h/c → 业务逻辑模块
│   ├── motor/ power/ comm/ hmi/
├── App/             # L5: 应用入口模板 + PID 调参协议
├── Config/          # YAML 配置（topologies 拓扑目录 + projects 工程 + params 参数变体）
├── YmaC/            # yaml_config_builder.py (GUI 配置注入) + scaffold.py (CLI 骨架生成)
├── docs/
│   ├── README.md    # 文档首页导航
│   ├── concept.md   # 思想哲学（唯一思想权威）
│   ├── coding/      # 编码方法论（oop + 采样-处理-发波 + 保护/HMI 线程）
│   ├── subsystems/  # 子系统参考（adc/pid/pll/pwm/... 各一文件）
│   ├── history/     # 开发历史（stage-* 每阶段一文件）+ lessons.md 教训库
│   ├── PLAN.md      # 唯一主线计划
│   └── learning/    # 外部项目学习报告
└── cmake/           # 工具链文件
```
> 目录分组表、MANIFEST schema、scaffold 工具规范见 [docs/PLAN.md](docs/PLAN.md) §一.3（已实现规格一句话 + 实现位置）。

**核心文档：**
| 文档 | 内容 |
|------|------|
| [docs/concept.md](docs/concept.md) | **设计原则（唯一思想权威）**：定位、三上下文、五层、系统层原语、复用 libxr 边界、远程注册表 |
| [agent.md](agent.md) | OOP 方法论、分层架构、虚函数表、继承/多态模式、6 子系统参考 |
| [docs/history/lessons.md](docs/history/lessons.md) | 调参教训库 (70 条 + 经验模板), git 版本管理, 禁止回退 |
| [docs/PLAN.md](docs/PLAN.md) | **唯一主线计划** — 已完成归档 + 未完成待办（含工具链/运行时/拓扑/系统层原语/远程分发） |

**App 模板：**
| 文件 | 用途 |
|------|------|
| [App/app_main.h.tmpl](App/app_main.h.tmpl) | App 层头文件模板 — 根结构体、配置 POD、ISR 钩子声明 |
| [App/app_main.c.tmpl](App/app_main.c.tmpl) | App 层实现模板 — board_init、YmaC 注入点、ISR 实现、BackgroundTask |
| [App/pid_tune.h/c](App/pid_tune.h) | PID 串口调参协议 (0xFB 帧, 48 字节) |
| [YmaC/README.md](YmaC/README.md) | YAML→C 配置注入工具使用说明 |

**AI 技能文件：**
| 技能 | 用途 |
|------|------|
| [.claude/skills/embedded-oop-c](.claude/skills/embedded-oop-c/SKILL.md) | C 语言 OOP 完整方法论 |
| [.claude/skills/c-code-style](.claude/skills/c-code-style/SKILL.md) | 代码风格：2空格缩进、K&R大括号、命名约定 |
| [.claude/skills/stm32-hal](.claude/skills/stm32-hal/SKILL.md) | STM32 HAL/LL 专业知识 |
| [.claude/skills/code-review-workflow](.claude/skills/code-review-workflow/SKILL.md) | **强制**写-审双Agent工作流 |
| [.claude/skills/project-orchestration](.claude/skills/project-orchestration/SKILL.md) | **项目统筹**：多 Agent 并发协作治理 — 用 RTOS 并发原语（线程/信号量/互斥锁/事件/队列/双缓冲/发布订阅）隐喻映射到 Agent 协作，防互相踩踏/撕裂/扯皮，含 plan 审批 + 角色边界 + 提交闸门 + git 状态纪律 |


## AI 助手工作准则（行为契约）

> 本节只约束 AI 的执行方式，与面向共同约定的 CLAUDE.md 互补：CLAUDE.md 面向人与 AI 双方，本节约束 AI 的执行方式。思想依据 [docs/concept.md](docs/concept.md)。

1. **计划先行** — 任何非平凡改动（新功能 / 重构 / 重组 / 涉及 3+ 文件）必须先给出 plan 并获批准，再动手实现。禁止大范围"边做边改"的自由发挥。
2. **每动作更新 HISTORY + LESSONS** — 每完成一个动作/阶段，同步更新 [docs/history/](docs/history/README.md)（阶段、commit 归因、错误与修正）与 [docs/history/lessons.md](docs/history/lessons.md)（新教训按经验模板，禁止回退）。文档落后于代码 = 违约。
3. **遵守代码风格** — 一律遵循 2 空格缩进、K&R 大括号、命名规范、include guard（见 [docs/coding/oop.md](docs/coding/oop.md) §6），提交前用 `.clang-format` / `.clang-tidy` 校验。
4. **Git 纪律** — 不自动提交；用户允许后**及时**按子系统拆分 commit（见下方 Git 约定）；提交前 `git status` 核对范围。
5. **写-审双 Agent** — 任何代码生成走强制 `code-review-workflow`（Writer → Reviewer），Reviewer 不可跳过（纯 .md / YAML 修改例外，commit 注明 `no-review: <原因>`）。
6. **库资产保护（误删库资产 = 最严重违规）** — HardC 是组件库不是可执行工程（见上方定位声明 + LESSONS.md #56）。未调用的组件/函数是**库存**，**禁止**以"无调用者 / 未被当前 App 或拓扑调用"为由删除、废弃或标注。删除/移动只允许两种情形：域归属重整（`git mv` 保留内容）或用户明确指令。Reviewer 不得把"未被调用"报为缺陷，检查清单见 code-review-workflow「库资产保护」节。


## 中断优先级三档（库级强制约定）

> **严格三档：FAST > SLOW > HMI，由 `bsp_irq_apply()` 默认强制（失败停机），不靠工程自觉。**
> 详见 [docs/coding/protection-hmi.md](docs/coding/protection-hmi.md) 与 [bsp_irq.h](BSP/bsp_irq.h)。

1. **FAST**（控制定时器 ISR）抢优先 **0**（最高）——采样→控制→发波→快保护。
2. **SLOW**（监控定时器 ISR）抢优先 **1**——慢保护/心跳/喂狗。
3. **CTX_HMI**（HMI/通信中断，`App_OnHmiTick`）抢优先 **2**——按键去抖/命令分发/收包入队。
4. 后台（BackgroundTask）主循环——慢 I/O（printf/OLED/协议/Flash）。
5. `bsp_irq_apply` 强制写 0/1/2 并全量读回校验；**返回 false = 误配，必须停机**，不允许带病运行。
6. 工程必须定义 `FAST_CTRL_IRQN` / `SLOW_CTRL_IRQN` / `HMI_IRQN` 三宏（STM32 IRQn / C2000 PIE 组占位）。
7. **禁止**把任何通信/HMI 中断优先级设到与 FAST 同级（0）——三档必须严格分离。

## Git 管理约定

### 分支策略

- `main` — 稳定分支，始终保持可构建、文档完整
- 功能/修复工作在 `feature/<name>` 或 `fix/<name>` 分支上进行，完成后合并回 `main`

### 提交规范

**提交粒度：每个子系统独立提交。** 修改 ADC 相关文件和 PWM 相关文件分开 commit，不要混在同一个提交里。提交信息格式：

```
<子系统或文件>: <动词短语>

# 示例：
ADC: 添加 AC Sampler 子类
PWM: 修复 Interleaved 相位计算溢出
Components: 更新 comp_error.h bitmask 宏
BSP: 添加 container_of.h
docs: 更新 agent.md 多态分发示例
cmake: 添加 gcc-arm-none-eabi 工具链文件
Config: 新增 default.yaml 配置模板
```

**提交信息用英文或中文均可，但同一仓库保持一致。** 本仓库优先用英文（面向国际协作），必要时可用中文附注。

### .gitignore 策略

已配置 `.gitignore`，忽略以下但不过分激进：

| 忽略 | 原因 |
|------|------|
| `*.o *.elf *.bin *.hex *.map` | 构建产物，每次编译重新生成 |
| `build/ cmake-build-*/` | CMake 输出目录 |
| `.vscode/ .idea/` | IDE 配置因人而异 |
| `.DS_Store Thumbs.db` | OS 垃圾文件 |
| `__pycache__/ *.pyc` | Python 缓存（YmaC 工具） |
| `.claude/settings.json` | 本地 AI 权限配置 — 含本机路径与私有项目引用，不上传 GitHub |
| `docs/AGENT-SYNC.md` 等内部规划 | 双 AI 交接日志 / 重整计划，私有工作笔记 |
| `docs/learning/` 学习报告 | 私人学习笔记（分析其他战队项目，含本机路径） |

| 不忽略 | 原因 |
|--------|------|
| `*.yaml` (Config/) | 配置文件，是源码的一部分 |

### AI 助手操作 Git 的规则

> **以下规则适用于所有 AI 助手（Claude Code、Copilot、Cursor 等）在此仓库中的操作。**

1. **不要自动提交。** 除非用户明确要求提交，否则只做修改、不 commit。
2. **不要 force push。** `--force` 到 `main` 永远禁止。force push 到功能分支需要用户明确确认。
3. **提交前检查：**
   - 修改是否跨子系统？→ 拆成多个提交，每个子系统一个
   - 是否有未跟踪的构建产物？→ 先更新 `.gitignore`
   - 是否破坏了文档的一致性？→ agent.md 和代码同步更新
4. **永远不要提交这些内容：**
   - API 密钥、密码、token（`.claude/settings.json` 及其余 `.claude/` 敏感文件均不入库）
   - 编译产物（`.o`, `.elf`, `.bin`）
   - IDE 个人配置（`.vscode/`, `.idea/`）
5. **`git status` 先看一眼再动手。** 不确定该不该提交的文件，先问用户。
6. **commit message 用上述格式。** 不确定影响范围时，先 `git diff --stat` 搞清楚改了哪些子系统。

### AI 助手的代码生成规则

> **强制规则：任何代码生成必须走写-审双 Agent 工作流。详见 [code-review-workflow](.claude/skills/code-review-workflow/SKILL.md)。**

1. **Writer Agent** — 读取需求 → 读取现有代码 → 生成/修改文件
2. **Reviewer Agent** — 独立检查 Writer 输出 → 对照检查清单 → 报告通过/不通过
3. **绝不跳过 Reviewer。** 即使只改一行，也要有人复查。
4. **Reviewer 必须检查：** 2空格缩进（禁止4空格）、K&R大括号、container_of 下溯、ops 绑定、include guard、行尾无空白、无 tab。
5. **例外（可跳过 Reviewer）：** 纯 .md 文档修改、YAML 配置修改。commit message 中注明 `no-review: <原因>`。

### 代码风格自动化

项目配置了 `.clang-format` (2空格 / K&R / 120列) 和 `.clang-tidy` (命名检查 + bug检查)。提交前运行：

```bash
# 格式化全部 .c/.h
find . -name '*.h' -o -name '*.c' | xargs clang-format -i

# 静态分析
clang-tidy Components/pid/comp_pid.h -- -I BSP -I Components -I Devices
# (子类示例) clang-tidy Devices/pid/pid_standard.h -- -I BSP -I Components -I Devices
```

### 人类协作者的 Git 工作流

```bash
# 开始新功能
git checkout -b feature/my-new-device
# ... 开发 ...
git add <相关文件>
git commit -m "PWM: 添加 SVPWM 六开关子类"

# 完成后合并
git checkout main
git merge feature/my-new-device
git branch -d feature/my-new-device
```

### 子系统独立性

所有子系统通过文件前缀区分（ADC/COM/Peripheral/PID/PWM/Motor），**文件按子系统归入 `Components/<域>/` + `Devices/<域>/` + `Module/<域>/` 子目录**，每个子目录含 `MANIFEST.yaml` 自描述（依赖声明）。修改子系统内部文件时，确认 [agent.md](agent.md) 中对应的 §8 子系统参考不需要同步更新。如果改了 API、增加了子类、修改了 ops 虚表签名，**必须同步更新文档**，并检查该子系统的 `MANIFEST.yaml` 依赖是否需要增删。

## App 层开发规则

> **App 层只有一组文件。禁止自由发挥。** 所有项目必须基于 [App/app_main.c.tmpl](App/app_main.c.tmpl) 和 [App/app_main.h.tmpl](App/app_main.h.tmpl) 开始。
> 参考: `User/app/app_main.c` (263行) + CMake `User/Application/app_main.c` (311行)。

1. **根结构体 `ProjectRoot`** — 嵌入所有 Device + Module 实例（值包含，零 malloc）
2. **配置 POD `ProjectConfig`** — 纯数据结构，YmaC 注入目标，与运行时 Instance 分离
3. **`board_init()`** — 唯一创建和绑定实例的地方：Device init → Module init → 指针注入 → ISR 启动
4. **`apply_config()`** — 配置 POD → 运行时 Instance 同步（启动时 + YAML 注入后 + 0xFB 调参后）
5. **`App_OnControlTick()`** — ISR 调用，顺序: 传感器→HMI→控制算法→执行器。禁止 printf
6. **`BackgroundTask()`** — 主循环，做所有耗时 I/O: printf、软件 I2C、OLED、串口应答
7. **`extern ProjectRoot g_root`** — Module 可通过 extern 访问兄弟模块（`extern Car car` 同款用法）

## YmaC 配置注入

```bash
# GUI 模式（含拓扑选择器）
cd <项目根目录>
python YmaC/yaml_config_builder.py

# CLI 模式
python YmaC/yaml_config_builder.py --cli default
```

**拓扑选择器（GUI Tab2）：** 扫 `Config/topologies/<topo>.yaml` 拓扑目录（buck/boost/forward/flyback/buckboost/sepic/cuk/zeta/buck2/vsi_3ph，`status: ready` 才可生成）→ 选拓扑 → 填工程名/MCU → 合成 `Config/projects/<name>.yaml` 并调 scaffold 生成 `build/gen/<name>/` → 参数表编辑（schema 来自拓扑 `params:`）→ 写 `Config/params/<name>_<variant>.yaml` → 注入物化 `app_main.c` → 编译。运行时调参（Tab3）走 0xFB 帧（`pip install pyserial`），槽位与离线 `params.slot` 同源。详见 [YmaC/README.md](YmaC/README.md)。

**工作流:**
1. 在 `app_main.c` 的 `/* CONFIG BEGIN */` / `/* CONFIG END */` 之间手写默认值
2. 创建 `Config/params/<variant>.yaml`（格式见 [YmaC/README.md](YmaC/README.md)）
3. 运行 YmaC → 选择配置 → 自动注入 → 编译
4. `apply_config()` 将注入的 POD 值同步到运行时 Instance

**YAML→C 类型映射:** 全大写字符串→C标识符, float→`.10f`, dict→designated initializer, list→C数组。

## 工程骨架生成 (scaffold.py)

> **新项目从 `Config/projects/<project>.yaml` 起步。** 声明式列出需要的子系统，工具自动解析依赖、生成构建文件与 App 骨架。规范见 [docs/PLAN.md](docs/PLAN.md) §一.3。

```bash
# 校验全部 MANIFEST.yaml（结构 + 依赖合法性）
python YmaC/scaffold.py scan

# 查看某子系统传递闭包依赖（拓扑排序 BSP→…→App）
python YmaC/scaffold.py deps components/pwm

# 从工程配置生成骨架 → build/gen/<project>/
python YmaC/scaffold.py gen Config/projects/<project>.yaml
```

**生成产物**（`build/gen/<project>/`）:
- `CMakeLists.txt` — 传递闭包内全部 .c 源文件 + 各层 include 路径（扁平 `#include` 无需改动）
- `<project>_deps.h` — 按层分组的依赖头文件清单
- `board_init_stub.c` — 每模块 `// TODO: <文件> 在此实例化` 占位，App 层填入真实初始化

## 构建

```bash
# 拓扑级自检: scaffold gen 产出 build/gen/<project>/CMakeLists.txt（闭包全部 .c + include 路径）
python YmaC/scaffold.py gen Config/projects/<project>.yaml
cmake -S build/gen/<project> -B build/out/<project> -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake  # GCC
# ARM Clang: -DCMAKE_TOOLCHAIN_FILE=cmake/starm-clang.cmake
cmake --build build/out/<project> -j$(nproc)

# 外部工程集成: include(${HARDC_DIR}/cmake/HardC.CMake)（YmaC cmake_integrate 注入外部工程 CMakeLists）
```

## BSP 硬件加速抽象层

> **Components 层禁止直接 include 平台加速库。** 所有硬件加速 (DSP/PWM/ADC) 通过 `BSP/` 层的不透明句柄接口分发。
> 详见 [docs/history/lessons.md](docs/history/lessons.md) #17 和 [agent.md](agent.md) §3.5.

| 文件 | 内容 |
|------|------|
| [BSP/bsp_dsp.h](BSP/bsp_dsp.h) | 硬件加速抽象 — sqrt/biquad, 平台检测 (CMSIS-DSP/C2000Ware/纯C回退) |
| [BSP/bsp_dsp_fir.h](BSP/bsp_dsp_fir.h) | FIR 滤波器硬件加速 (4 后端: CMSIS-DSP/C2000/纯C/Q15 定点) |
| [BSP/bsp_dsp_fft.h](BSP/bsp_dsp_fft.h) | FFT 快速傅里叶变换 (RFFT + CFFT, 3 后端) |
| [BSP/bsp_pwm.h](BSP/bsp_pwm.h) | PWM 不透明句柄 + 物理参数 API (duty/Hz/ns/deg) |
| [BSP/bsp_adc.h](BSP/bsp_adc.h) | ADC 校准/启动抽象 |

## 复用方式

1. **直接拷贝需要的子系统目录**（`Components/<域>/` + `Devices/<域>/` + `Module/<域>/`，连同 `MANIFEST.yaml`）到目标工程，或用 `scaffold.py gen` 从 `project.yaml` 生成骨架
2. **基于父类继承新子类** — 遵循 agent.md 中的黄金法则（父类为第一成员、container_of 下溯、构造器绑定 ops）

## 子系统速查

> **每个域一个子目录**：`Components/<域>/`（父类）+ `Devices/<域>/`（子类）+ `Module/<域>/`（业务逻辑）。各目录文件清单与依赖见其 `MANIFEST.yaml`。

| 域 | Component | Devices 子类数 | 句柄头文件 |
|----|-----------|--------------|-----------|
| ADC | `comp_adc.h/c` | 3 (Follower/DC/AC Sampler) | `adcs.h` |
| COM | `comp_comm.h/c`（契约基类: 名称 + 诊断 ops self_check/reset, 数据面不虚化）+ `comp_dma_rx.h/comp_dma_tx.h`（DMA model, 对标 libxr） | 5 (Uart/Spi/I2c/Can/Gpio) + 协议模块 (mod_comm/mod_cmd_dispatch/mod_serial_proto/mod_can_proto/mod_pmbus, 全跑 CTX_HMI) — **BSP 之上零 HAL, 非阻塞** (UART DMA / SPI/I2C 中断事务 / CAN FIFO 中断+发送队列; SW I2C 已删) | — |
| Peripheral | `comp_output.h/c` (OutputBase 开关类) + `comp_sensor.h/c` (SensorBase 测量类) + `comp_mpu.h` (+ `comp_mpu_dmp.c`) | 8 (Led/Laser/Beep/Buzzer/Fan/Oled/Mpu6050/Ultrasonic; OLED 无基类, 设备=域基类+Gpio/PWM/UART 总线组合, 全 HAL-free) | `pers.h` |
| PID | `comp_pid.h/c` (父类 PidBase + PidOps, 纯契约) + `comp_tcm.h` (自动调参) | 14 (Standard 串行 / Parallel 并行 / DCL 2-DOF+D滤波(含grando) / Pi 条件PI(原名 Solar) / Reg4 设定值滤波+前馈 / Reg3 反计算+位置wrap / PR 理想谐振 / QPR 准谐振 / P2PD 非线性 / P 纯比例 / I 纯积分 / PD 比例+微分 / NL 幂律整形 / Cascade 级联) | `pids.h` |
| PLL | `comp_pll_base.h/c` (父类 PllBase + PllOps, PD→LF→VCO 三环节, 一个输入帧+一个反馈输出, LF(PI)+VCO 下沉基类) | 5 (Sogi SOGI正交+Park投影(含SSRF-SPLL) / Srf Park旋转投影 / Notch 纯乘法+陷波 / Ddsrf 正负序解耦 / SogiFll SOGI+FLL频率自适) | `plls.h` |
| PWM | `comp_pwm.h/c` + `comp_sgen.h` (正弦发生器) | 9 (BuckBoost/HalfBridge/FullBridge/Interleaved/Resonant/SEPIC/SPWM/SVPWM/WPT) | `pwms.h` |
| Motor | `comp_motor.h/c` + `comp_bldc_instaspin.h` (无感FOC) + `comp_mod6.h` (模6换相) + 无感观测器家族 (SMO/eSMO/HFI/ACI/Resolver/SVGEN/CURMOD/VHz/速度角度/电压) | 2 (TIM + Encoder 位置编码器, 非 MotorBase 子类 — 独立结构体组合 Uart/Gpio/ADC 总线) | — |
| StepMotor | `comp_step_motor.h/c` | 1 (motor_step) | — |
| Codec | `comp_crc.h` / `comp_checksum.h` / `comp_endian.h` / `comp_viterbi.h` / `comp_interleaver.h` / `comp_rs.h` / `comp_ask.h` | 7 (CRC/Checksum/Endian/Viterbi/Interleaver/RS/ASK) | — |
| Contract | `comp_io.h` (I/O 完成契约) + `comp_double_buffer.h` / `comp_latch.h` / `comp_ring.h` / `comp_mailbox.h` (五原语跨上下文交接) | — (独立) | — |
| Math | `comp_math.h`（**全库唯一 π/2π float 常量源 M_PI/M_2PI + 硬件加速宏 MATH_SQRT/ISQRT/ABS，工程可覆盖**）+ `comp_iqmath.h/c` + `comp_error.h` + `comp_complex.h` | — (独立) | — |

**独立 Component（无 Devices 层, 单头文件 static inline，位于 `Components/dsp/`、`power/`、`math/`、`codec/`、`motor/`、`pid/`、`contract/`）：**

| Component | 用途 |
|-----------|------|
| `comp_dlog.h` | 数据记录器 (Dlog1ch/Dlog4ch) — 环形缓冲 + 触发 + 预分频 |
| `comp_vector.h` | 向量/矩阵批量运算 (add/sub/mul/dot/absmax/clamp + Vector3) |
| `comp_pfc.h` | PFC 功率因数校正 — 电流指令 + 无桥 PFC (PfcBlIcmd) + RMS² 倒数 |
| `comp_complex.h` | 复数运算 (Complex) — 加减乘除/共轭/模/幅角/极坐标转换 |
| `comp_math.h` | 数学工具 — **全库唯一 π/2π float 常量源 (M_PI/M_2PI, `#undef` 防系统 `<math.h>` double 泄漏)** + 硬件加速宏 (MATH_SQRT/MATH_ISQRT/MATH_ABS, 默认走 bsp_sqrt_f32/bsp_isqrt_f32/fabsf, 工程可 `#define` 覆盖) + 限幅/绝对值/死区/线性映射 |
| `comp_crc.h` | CRC 校验 — 8/16/32 位循环冗余校验, 查表法 + 比特流 |
| `comp_checksum.h` | 校验和 — 8/16/32 位无符号累加 (自然溢出), static inline, ISR 安全 |
| `comp_endian.h` | 大小端转换 — 16/32 位原地翻转 + 双缓冲拷贝, static inline |
| `comp_viterbi.h` | Viterbi 解码器 — 卷积码最大似然译码, 分支度量 + 回溯 |
| `comp_interleaver.h` | 交织器 — 块交织/解交织 (行列交织器), 地址生成 |
| `comp_rs.h` | RS 编解码 — Reed-Solomon 纠错码, Berlekamp-Massey + Forney 算法 |
| `comp_ask.h` | ASK/OOK 无线充电信令 — 12-bit 包 (req+功率+偶校验) 编解码 + 2000Hz 包络解码状态机 |
| `comp_bldc_instaspin.h` | BLDC InstaSPIN — 无传感器 FOC, FAST 观测器 + 磁链/转矩估计 |
| `comp_impulse.h` | 脉冲发生器 — 每 Period 采样输出满幅脉冲 (0x7FFF) |
| `comp_mod6.h` | 模 6 换相计数器 — BLDC 六步换相步进 (0→5→0) |
| `comp_power_meas.h` | 电力测量 — 真有效值/有功/无功/视在功率/功率因数/相位角 + 能量脉冲积分 (残余结转) + 三相聚合 (总功率/线电压/电流矢量和) |
| `comp_power_goertzel.h` | Goertzel 逐谐波频谱 (H1..H50) + THD — 整数周期窗口谐振器, 无需窗函数 |
| `comp_power_calib.h` | 结果级校准 POD — 死区减法 (保符号对称死区), 即 TI NV 持久化结构体 |
| `comp_power_event.h` | 电压事件检测 — 暂降/暂升/中断状态机 + 事件计数/时长 (滞回 + 交叉检测) |
| `comp_power_fund.h` | 基波电力分析 — 同步正交相关解调 (IEC 62053), 基波有效值/有功/无功 + THD, 对谐波污染免疫 |
| `comp_protection.h` | 保护框架 — 阈值检测、去抖、分级响应 (Components/protection/ 域) |
| `comp_protection_3lvl.h` | 三电平逆变器延迟保护 — 主开关立即关断 + 内开关故障消隐延迟关断 (ride-through, 非锁存自重新布防) |
| `comp_database.h/c` | 闪存键值数据库 — 主备双块 + 顺序写入, 参数/校准持久化 (PLAN §2.1 系统层原语, `Components/database/`, FlashOps 访问, 可 host 单测) |

**Module 层 (L4) — 业务逻辑（位于 `Module/motor/`、`power/`、`comm/`、`hmi/`）：**
| 模块 | 文件 | 用途 |
|------|------|------|
| MotApp | `mod_motor.h/c` | 单电机状态机 (IDLE/SPD/POS/SP) + 超时保护 |
| TurnCtrl | `mod_turn.h/c` | 编码器 tick 计数转弯 (UTURN/LTURN/RTURN) |
| Follower | `mod_follower.h/c` | P2PD 循迹 (8路红外→差速修正) |
| HMI | `mod_hmi.h/c` | 按键去抖 + OLED 菜单 + 命令分发 + 输出端口路由 (唯一路由决策点: HmiPorts {can,uart,led} fan-out, 回调非阻塞) |
| ModBalance | `mod_balance.h/c` | 球板平衡 (步进电机+PID, 骨架) |
| CmdDispatch | `mod_cmd_dispatch.h/c` | 统一命令分发 (按键/串口→CarCmd→回调) |
| ModComm | `mod_comm.h/c` | 帧协议解析 (0xAA 帧头; send_fn 回调接缝, 未绑定则忽略) |
| SerialProto | `mod_serial_proto.h/c` | 调试串口协议 (0xFA/FC/EE/EF + PID 调参) |
| SFRA | `mod_sfra.h/c` | 软件频响分析仪 — DDS扰动注入 + DFT采集 + Bode图输出 |
| FCL | `mod_fcl_ctrl.h/c` | 快速电流环 — dq 轴双环 PI + 交叉解耦 + 反电动势前馈 |
| PMBus | `mod_pmbus.h/c` | PMBus 协议栈 — SMBus 2.0 + PMBus 1.3 命令集 (I2C 从机) |
| SuperCap | `mod_supercap.h/c` | 超级电容功率环 — 级联功率控制 + 满电停充/低压切除 + 短路/失平衡保护 (PowerStage 派生) |
| CurrentShare | `mod_current_share.h/c` | 三相均流 — 模式迟滞 + 逐相电流均衡 + 切换同步 (N=1..3) |
| CanProto | `mod_can_proto.h/c` | 超级电容 CAN 协议 — 0x051 遥测 / 0x061 裁判功率 (ctx main) |

---

> **最后更新：** 2026-08-16 — 通信层 DMA/中断事务改造完成: BSP 之上零 HAL（bsp_uart/i2c/spi/can 五抽象 × stm32/c2000 后端）+ comp_dma_rx/tx DMA model（对标 libxr）+ com_uart/i2c/spi/can 全部接入（UART DMA / SPI/I2C 中断事务 / CAN FIFO 中断+发送队列, SW I2C 删除）+ 中断优先级三档（bsp_irq 库级强制 FAST>SLOW>HMI + CTX_HMI 钩子）。此前: PID 域重构完成 (父类 PidBase + 14 子类 + PLL 域 PllBase + 5 子类); comm 子系统重构 (阶段 0-4) 见 stage-22。
