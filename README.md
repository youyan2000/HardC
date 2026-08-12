# C-OOP — 嵌入式 C 面向对象硬件驱动框架

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
| Motor | `comp_motor`, `comp_step_motor` | `motor_tim`, `motor_step` | `mod_motor`, `mod_turn`, `mod_follower`, `mod_balance`, `mod_fcl_ctrl` |
| VCU | `comp_complex`, `comp_crc`, `comp_viterbi`, `comp_interleaver`, `comp_rs` | — | `mod_pmbus` |
| DSP | `comp_esmo`, `comp_hfi`, `comp_smo`, `comp_dlog`, `comp_vector`, `comp_pi_reg4`, `comp_bldc_instaspin` | — | `mod_sfra` |

独立 Component（单头文件，无 Devices 层）覆盖 PFC、坐标变换、PLL、MPPT、滤波器、FFT 窗函数、信号发生器、速度估计等算法领域，详见 [agent.md](agent.md) 独立 Component 表。

## 快速开始

```bash
# 1. 选择或创建拓扑配置
cp Config/topologies/supercap_3ph.yaml Config/params/my_config.yaml

# 2. YAML → C 注入
python YmaC/yaml_config_builder.py --cli my_config

# 3. 编译
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/starm-clang.cmake
make -j$(nproc)
```

## 复用

1. **拷贝文件** — 复制需要的 Component + Device + Module 到目标工程
2. **继承新子类** — 父类为第一成员 → 实现 ops → container_of 下溯 → 绑定

## 文档

| 文档 | 内容 |
|------|------|
| [CLAUDE.md](CLAUDE.md) | 项目入口：Git 约定、代码生成规则、App 架构 |
| [agent.md](agent.md) | OOP 方法论：虚函数表、继承/多态、6 子系统参考 |
| [LESSONS.md](LESSONS.md) | 44 条调参教训 |
| [HISTORY.md](HISTORY.md) | 项目完整历程 |
| [ROADMAP.md](ROADMAP.md) | 多拓扑构建系统路线图 |
| [docs/](docs/) | 参考项目学习报告 |
