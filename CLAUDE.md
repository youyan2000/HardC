# CLAUDE.md — C-OOP 项目入口（AI + 人类共读）

> 本文件是 AI 助手和人类协作者的共同入口。它记录项目级别的约定，不重复代码本身已有的信息。

## 项目概述

纯 C 语言实现的面向对象嵌入式硬件驱动框架。5 层扁平目录：BSP → Components → Devices → Module → App。文件前缀区分子系统域（ADC/COM/GPO/PID/PWM/Motor）。跨平台：STM32 (HAL/HRTIM) + TI C2000 (ePWM/CLA) + 纯C回退。

**目录结构：**
```
C-OOP/
├── BSP/           # L1: 不透明句柄 → 平台抽象
├── Components/    # L2: comp_*.h/c → 父类 + ops 虚表（前缀=域）
├── Devices/       # L3: <域>_<子类>.h/c → 具体硬件实现
├── Module/        # L4: mod_*.h/c → 业务逻辑模块
├── App/           # L5: 应用入口模板 + PID 调参协议
├── Config/        # YAML 配置（拓扑 + 参数变体）
├── YmaC/          # YAML→C 注入工具
├── docs/          # 学习报告 & 架构文档
└── cmake/         # 工具链文件
```

**核心文档：**
| 文档 | 内容 |
|------|------|
| [agent.md](agent.md) | OOP 方法论、分层架构、虚函数表、继承/多态模式、6 子系统参考 |
| [LESSONS.md](LESSONS.md) | 调参教训库 (44 条 + 经验模板), git 版本管理, 禁止回退 |
| [ROADMAP.md](ROADMAP.md) | 多拓扑构建系统路线图 |

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
| [.claude/skills/eugene-code-style](.claude/skills/eugene-code-style/SKILL.md) | 代码风格：2空格缩进、K&R大括号、命名约定 |
| [.claude/skills/stm32-hal](.claude/skills/stm32-hal/SKILL.md) | STM32 HAL/LL 专业知识 |
| [.claude/skills/code-review-workflow](.claude/skills/code-review-workflow/SKILL.md) | **强制**写-审双Agent工作流 |

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

| 不忽略 | 原因 |
|--------|------|
| `.claude/settings.json` | 项目级 AI 工具配置，团队共享 |
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
   - API 密钥、密码、token（包括 `.claude/` 中非 settings.json 的敏感文件）
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
clang-tidy Components/comp_pid.h -- -I BSP -I Components -I Devices
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

所有子系统通过文件前缀区分（ADC/COM/GPO/PID/PWM/Motor），全部在 `Components/` + `Devices/` + `Module/` 扁平存放。修改子系统内部文件时，确认 [agent.md](agent.md) 中对应的 §8 子系统参考不需要同步更新。如果改了 API、增加了子类、或修改了 ops 虚表签名，**必须同步更新文档**。

## App 层开发规则

> **App 层只有一组文件。禁止自由发挥。** 所有项目必须基于 [App/app_main.c.tmpl](App/app_main.c.tmpl) 和 [App/app_main.h.tmpl](App/app_main.h.tmpl) 开始。
> 参考: WEILAI_SuperCap `User/app/app_main.c` (263行) + LitteCar CMake `User/Application/app_main.c` (311行)。

1. **根结构体 `ProjectRoot`** — 嵌入所有 Device + Module 实例（值包含，零 malloc）
2. **配置 POD `ProjectConfig`** — 纯数据结构，YmaC 注入目标，与运行时 Instance 分离
3. **`board_init()`** — 唯一创建和绑定实例的地方：Device init → Module init → 指针注入 → ISR 启动
4. **`apply_config()`** — 配置 POD → 运行时 Instance 同步（启动时 + YAML 注入后 + 0xFB 调参后）
5. **`App_OnControlTick()`** — ISR 调用，顺序: 传感器→HMI→控制算法→执行器。禁止 printf
6. **`BackgroundTask()`** — 主循环，做所有耗时 I/O: printf、软件 I2C、OLED、串口应答
7. **`extern ProjectRoot g_root`** — Module 可通过 extern 访问兄弟模块（参考 LitteCar `extern Car car`）

## YmaC 配置注入

```bash
# GUI 模式
cd <项目根目录>
python YmaC/yaml_config_builder.py

# CLI 模式
python YmaC/yaml_config_builder.py --cli default
```

**工作流:**
1. 在 `app_main.c` 的 `/* CONFIG BEGIN */` / `/* CONFIG END */` 之间手写默认值
2. 创建 `Config/params/<variant>.yaml`（格式见 [YmaC/README.md](YmaC/README.md)）
3. 运行 YmaC → 选择配置 → 自动注入 → 编译
4. `apply_config()` 将注入的 POD 值同步到运行时 Instance

**YAML→C 类型映射:** 全大写字符串→C标识符, float→`.10f`, dict→designated initializer, list→C数组。

## 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/starm-clang.cmake  # ARM Clang
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake  # GCC
make -j$(nproc)
```

## BSP 硬件加速抽象层

> **Components 层禁止直接 include 平台加速库。** 所有硬件加速 (DSP/PWM/ADC) 通过 `BSP/` 层的不透明句柄接口分发。
> 详见 [LESSONS.md](LESSONS.md) #17 和 [agent.md](agent.md) §3.5.

| 文件 | 内容 |
|------|------|
| [BSP/bsp_dsp.h](BSP/bsp_dsp.h) | 硬件加速抽象 — sqrt/biquad, 平台检测 (CMSIS-DSP/C2000Ware/纯C回退) |
| [BSP/bsp_dsp_fir.h](BSP/bsp_dsp_fir.h) | FIR 滤波器硬件加速 (4 后端: CMSIS-DSP/C2000/纯C/Q15 定点) |
| [BSP/bsp_dsp_fft.h](BSP/bsp_dsp_fft.h) | FFT 快速傅里叶变换 (RFFT + CFFT, 3 后端) |
| [BSP/bsp_pwm.h](BSP/bsp_pwm.h) | PWM 不透明句柄 + 物理参数 API (duty/Hz/ns/deg) |
| [BSP/bsp_adc.h](BSP/bsp_adc.h) | ADC 校准/启动抽象 |

## 复用方式

1. **直接拷贝需要的 Component + Device + Module 文件** 到目标工程
2. **基于父类继承新子类** — 遵循 agent.md 中的黄金法则（父类为第一成员、container_of 下溯、构造器绑定 ops）

## 子系统速查

| 域 | Component | Devices 子类数 | 句柄头文件 |
|----|-----------|--------------|-----------|
| ADC | `comp_adc.h/c` | 3 (Follower/DC/AC Sampler) | `adcs.h` |
| COM | `comp_comm.h/c` | 9 (UART/SPI/I2C/CAN/Key/MPU6050/OLED/Ultrasonic/Encoder) | `comms.h` |
| GPO | `comp_gpo.h/c` | 5 (LED/Laser/Beep/Buzzer/Fan) | `gpos.h` |
| PID | `comp_pid.h/c` + `comp_pi_reg4.h` (四态PI) | 9 (Standard/Cascade/P2PD/Parallel/PR/QPR/DCL/Grando/Solar) | `pids.h` |
| PWM | `comp_pwm.h/c` + `comp_sgen.h` (正弦发生器) | 6 (BuckBoost/HalfBridge/FullBridge/Interleaved/Resonant/SVPWM) | `pwms.h` |
| Motor | `comp_motor.h/c` + `comp_bldc_instaspin.h` (无感FOC) | 1 (TIM) | — |
| StepMotor | `comp_step_motor.h/c` | 1 (motor_step) | — |
| VCU | `comp_complex.h` / `comp_crc.h` / `comp_viterbi.h` / `comp_interleaver.h` / `comp_rs.h` | 5 (Complex/CRC/Viterbi/Interleaver/RS) | — |

**独立 Component（无 Devices 层, 单头文件 static inline）：**

| Component | 用途 |
|-----------|------|
| `comp_esmo.h` | 增强滑模观测器 (eSMO) — PLL 锁相环角度/速度估计 (PMSM/BLDC) |
| `comp_dlog.h` | 数据记录器 (Dlog1ch/Dlog4ch) — 环形缓冲 + 触发 + 预分频 |
| `comp_vector.h` | 向量/矩阵批量运算 (add/sub/mul/dot/absmax/clamp + Vector3) |
| `comp_pfc.h` | PFC 功率因数校正 — 电流指令 + 无桥 PFC (PfcBlIcmd) + RMS² 倒数 |
| `comp_complex.h` | 复数运算 (Complex) — 加减乘除/共轭/模/幅角/极坐标转换 |
| `comp_crc.h` | CRC 校验 — 8/16/32 位循环冗余校验, 查表法 + 比特流 |
| `comp_viterbi.h` | Viterbi 解码器 — 卷积码最大似然译码, 分支度量 + 回溯 |
| `comp_interleaver.h` | 交织器 — 块交织/解交织 (行列交织器), 地址生成 |
| `comp_rs.h` | RS 编解码 — Reed-Solomon 纠错码, Berlekamp-Massey + Forney 算法 |
| `comp_pi_reg4.h` | 四态 PI 调节器 — 带抗饱和的 PI 控制 (正常/上限/下限/跟踪) |
| `comp_bldc_instaspin.h` | BLDC InstaSPIN — 无传感器 FOC, FAST 观测器 + 磁链/转矩估计 |

**Module 层 (L4) — 业务逻辑:**
| 模块 | 文件 | 用途 |
|------|------|------|
| MotApp | `mod_motor.h/c` | 单电机状态机 (IDLE/SPD/POS/SP) + 超时保护 |
| TurnCtrl | `mod_turn.h/c` | 编码器 tick 计数转弯 (UTURN/LTURN/RTURN) |
| Follower | `mod_follower.h/c` | P2PD 循迹 (8路红外→差速修正) |
| HMI | `mod_hmi.h/c` | 按键去抖 + OLED 菜单 + 命令分发 |
| ModBalance | `mod_balance.h/c` | 球板平衡 (步进电机+PID, 骨架) |
| CmdDispatch | `mod_cmd_dispatch.h/c` | 统一命令分发 (按键/串口→CarCmd→回调) |
| ModComm | `mod_comm.h/c` | 帧协议解析 (0xAA 帧头) |
| SerialProto | `mod_serial_proto.h/c` | 调试串口协议 (0xFA/FC/EE/EF + PID 调参) |
| SFRA | `mod_sfra.h/c` | 软件频响分析仪 — DDS扰动注入 + DFT采集 + Bode图输出 |
| FCL | `mod_fcl_ctrl.h/c` | 快速电流环 — dq 轴双环 PI + 交叉解耦 + 反电动势前馈 |
| PMBus | `mod_pmbus.h/c` | PMBus 协议栈 — SMBus 2.0 + PMBus 1.3 命令集 (I2C 从机) |

---

> **最后更新：** 2026-08-12 — 多 Agent 并行移植 controlSUITE 6 算法 (eSMO/DLOG/Vector/PFC/SVPWM-DPWM/SFRA) + 文档同步更新