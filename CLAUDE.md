# CLAUDE.md — STM32_OOP 项目入口（AI + 人类共读）

> 本文件是 AI 助手和人类协作者的共同入口。它记录项目级别的约定，不重复代码本身已有的信息。

## 项目概述

纯 C 语言实现的面向对象 STM32 硬件驱动框架。5 层架构：BSP → Components → Devices → Module → App。每个子项目（ADC-OOP, COM-OOP, GPO-OOP, PID-OOP, PWM-OOP）是独立可复用的层级模块。

**核心文档：**
- [agent.md](agent.md) — OOP 方法论、分层架构、虚函数表、继承/多态模式
- [ANALYSIS.md](ANALYSIS.md) — 三项目对比分析 (STM32_OOP × LitteCar × RM) + 16 条实战教训

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
