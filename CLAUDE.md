# CLAUDE.md — STM32_OOP 项目入口（AI + 人类共读）

> 本文件是 AI 助手和人类协作者的共同入口。它记录项目级别的约定，不重复代码本身已有的信息。

## 项目概述

纯 C 语言实现的面向对象 STM32 硬件驱动框架。5 层架构：BSP → Components → Devices → Module → App。每个子项目（ADC-OOP, COM-OOP, GPO-OOP, PID-OOP, PWM-OOP）是独立可复用的层级模块。

**核心文档：**
| 文档 | 内容 |
|------|------|
| [agent.md](agent.md) | OOP 方法论、分层架构、虚函数表、继承/多态模式、App 架构规则 |
| [LESSONS.md](LESSONS.md) | 调参教训库 (16 条 + 经验模板), git 版本管理, 禁止回退 |

**App 模板：**
| 文件 | 用途 |
|------|------|
| [Templates/app_main.h.tmpl](Templates/app_main.h.tmpl) | App 层头文件模板 — 根结构体、配置 POD、ISR 钩子声明 |
| [Templates/app_main.c.tmpl](Templates/app_main.c.tmpl) | App 层实现模板 — board_init、YmaC 注入点、ISR 实现、BackgroundTask |
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

**提交粒度：每个子项目独立提交。** 修改 ADC-OOP 和 PWM-OOP 分开 commit，不要混在同一个提交里。提交信息格式：

```
<子项目或子系统>: <动词短语>

# 示例：
ADC-OOP: 添加 AC Sampler 子类
PWM-OOP: 修复 Interleaved 相位计算溢出
Components: 更新 comp_error.h bitmask 宏
BSP: 添加 container_of.h
docs: 更新 agent.md 多态分发示例
cmake: 添加 gcc-arm-none-eabi 工具链文件
conf: 新增 default.yaml 配置模板
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
| `*.yaml` (conf/) | 配置文件，是源码的一部分 |

### AI 助手操作 Git 的规则

> **以下规则适用于所有 AI 助手（Claude Code、Copilot、Cursor 等）在此仓库中的操作。**

1. **不要自动提交。** 除非用户明确要求提交，否则只做修改、不 commit。
2. **不要 force push。** `--force` 到 `main` 永远禁止。force push 到功能分支需要用户明确确认。
3. **提交前检查：**
   - 修改是否跨子项目？→ 拆成多个提交，每个子项目一个
   - 是否有未跟踪的构建产物？→ 先更新 `.gitignore`
   - 是否破坏了文档的一致性？→ agent.md 和代码同步更新
4. **永远不要提交这些内容：**
   - API 密钥、密码、token（包括 `.claude/` 中非 settings.json 的敏感文件）
   - 编译产物（`.o`, `.elf`, `.bin`）
   - IDE 个人配置（`.vscode/`, `.idea/`）
5. **`git status` 先看一眼再动手。** 不确定该不该提交的文件，先问用户。
6. **commit message 用上述格式。** 不确定影响范围时，先 `git diff --stat` 搞清楚改了哪些子项目。

### AI 助手的代码生成规则

> **强制规则：任何代码生成必须走写-审双 Agent 工作流。详见 [code-review-workflow](.claude/skills/code-review-workflow/SKILL.md)。**

1. **Writer Agent** — 读取需求 → 读取现有代码 → 生成/修改文件
2. **Reviewer Agent** — 独立检查 Writer 输出 → 对照检查清单 → 报告通过/不通过
3. **绝不跳过 Reviewer。** 即使只改一行，也要有人复查。
4. **Reviewer 必须检查：** 2空格缩进（禁止4空格）、K&R大括号、container_of 下溯、ops 绑定、include guard、行尾无空白、无 tab。
5. **例外（可跳过 Reviewer）：** 纯 .md 文档修改、YAML 配置修改。commit message 中注明 `no-review: <原因>`。

### 人类协作者的 Git 工作流

```bash
# 开始新功能
git checkout -b feature/my-new-device
# ... 开发 ...
git add <子项目目录>
git commit -m "GPO-OOP: 添加 RGB LED 子类"

# 完成后合并
git checkout main
git merge feature/my-new-device
git branch -d feature/my-new-device
```

### 子项目独立性的 Git 含义

每个子项目（ADC-OOP, COM-OOP, GPO-OOP, PID-OOP, PWM-OOP）有自己的 `agent.md`。修改子项目内部文件时，确认该子项目的 `agent.md` 不需要同步更新。如果改了 API、增加了子类、或修改了 ops 虚表签名，**必须同步更新文档**。

## App 层开发规则

> **App 层只有一组文件。禁止自由发挥。** 所有项目必须基于 [Templates/app_main.c.tmpl](Templates/app_main.c.tmpl) 和 [Templates/app_main.h.tmpl](Templates/app_main.h.tmpl) 开始。
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
2. 创建 `conf/<variant>.yaml`（格式见 [YmaC/README.md](YmaC/README.md)）
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

## 复用方式

1. **直接拷贝整个子项目目录** 到目标工程
2. **基于父类继承新子类** — 遵循 agent.md 中的黄金法则（父类为第一成员、container_of 下溯、构造器绑定 ops）

---

> **最后更新：** 2026-07-19 — Git 管理初始化
