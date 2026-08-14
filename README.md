# HardC — 嵌入式 C 面向对象的电力电子/电机控制硬件驱动框架

> 纯 C99 OOP | 五层扁平架构 | 多平台 (STM32 + C2000) | 文件前缀区分域

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

> 44 Components, 35+ Devices, 13 Modules, 8 BSP 文件

## 子系统

| 域 | Component | Devices | Module |
|----|-----------|---------|--------|
| ADC | `comp_adc` | `adc_follower`, `adc_dc_sampler`, `adc_ac_sampler` | `mod_sampler` |
| COM | `comp_comm` | `com_uart`, `com_spi`, `com_i2c`, `com_can`, `com_key`, `com_mpu6050`, `com_oled`, `com_ultrasonic`, `com_encoder` | `mod_comm`, `mod_cmd_dispatch`, `mod_serial_proto`, `mod_hmi` |
| GPO | `comp_gpo` | `gpo_led`, `gpo_laser`, `gpo_beep`, `gpo_buzzer`, `gpo_fan` | — |
| PID | `comp_pid` | `pid_standard`, `pid_cascade`, `pid_p2pd`, `pid_parallel`, `pid_pr`, `pid_qpr`, `pid_dcl`, `pid_grando`, `pid_solar` | — |
| PWM | `comp_pwm` | `pwm_buckboost`, `pwm_half_bridge`, `pwm_full_bridge`, `pwm_interleaved`, `pwm_resonant`, `pwm_sepic`, `pwm_svpwm` | `mod_powerctrl` |
| Power | `comp_power_stage` | — | `mod_buck`（PowerStage 示例，YmaC 拓扑选择器入口） |
| Motor | `comp_motor`, `comp_step_motor` | `motor_tim`, `motor_step` | `mod_motor`, `mod_turn`, `mod_follower`, `mod_balance`, `mod_fcl_ctrl` |
| VCU | `comp_complex`, `comp_crc`, `comp_viterbi`, `comp_interleaver`, `comp_rs` | — | `mod_pmbus` |
| DSP | `comp_esmo`, `comp_hfi`, `comp_smo`, `comp_dlog`, `comp_vector`, `comp_pi_reg4`, `comp_bldc_instaspin` | — | `mod_sfra` |

独立 Component（单头文件，无 Devices 层）覆盖 PFC、坐标变换、PLL、MPPT、滤波器、FFT 窗函数、信号发生器、速度估计等算法领域，详见 [agent.md](agent.md) 独立 Component 表。

## 快速开始

> 完整 GUI 走查见 [YmaC/README.md](YmaC/README.md) §3「验收走查」。

**选拓扑 → 生成工程 → 调参 → 注入 → 编译**（新项目的唯一入口）：

```bash
# 1. 启动 YmaC 拓扑选择器（GUI，项目根目录）
python YmaC\yaml_config_builder.py          # Windows
python YmaC/yaml_config_builder.py          # Linux

# 2. 在 Tab2「拓扑选择」：选拓扑（buck 是唯一 ready）→ 填工程名/MCU
#    → 生成工程（Config/projects/<name>.yaml + build/gen/<name>/）
#    → 参数表编辑 → 写入参数（Config/params/<name>_<variant>.yaml）
#    → 注入 App（物化 build/gen/<name>/app_main.c）→ 编译

# 3. 或纯 CLI：从工程 YAML 生成骨架（无 GUI 环境）
python YmaC/scaffold.py gen Config/projects/<name>.yaml

# 4. 手动编译（注入 App 后）
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/starm-clang.cmake
make -j$(nproc)
```

选完拓扑后只剩两件事：在 App 层接外部 I/O（采样输入 / PWM 输出 / HMI），然后用 YmaC 调参（离线 Tab1 注入 + 运行时 Tab3 0xFB 串口帧）。

## 复用

1. **拷贝文件** — 复制需要的 Component + Device + Module 到目标工程
2. **继承新子类** — 父类为第一成员 → 实现 ops → container_of 下溯 → 绑定

## 文档

| 文档 | 内容 |
|------|------|
| [CLAUDE.md](CLAUDE.md) | 共同约定：Git 约定、代码生成规则、App 架构（人与 AI 共读） |
| [agent.md](agent.md) | AI 行为准则 + OOP 方法论：虚函数表、继承/多态、6 子系统参考 |
| [docs/debug/LESSONS.md](docs/debug/LESSONS.md) | 58 条调参教训 |
| [docs/debug/history/](docs/debug/history/README.md) | 项目完整历程 |
| [docs/debug/ROADMAP.md](docs/debug/ROADMAP.md) | 多拓扑构建系统路线图 |
| [docs/learning/](docs/learning/) | 学习总结资料（外部项目学习报告 + 架构原则） |
| [docs/debug/](docs/debug/) | 记录和计划（历史、教训、路线图、设计文档） |
