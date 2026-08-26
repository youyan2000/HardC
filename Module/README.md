# Module —— 业务逻辑层

HardC 五层架构 L4。这一层是"一件件事"：控制策略、状态机、协议命令——通过指针注入组装 Components/Devices，自身不碰硬件。

## 这层干什么

- 一个模块 = 一件"事"：按目录归位 + 前缀命名（见下表），如 `gen_buck` / `gen_supercap` / `hmi_comm` / `prot_monitor`。
- 命名规则：文件名前缀 = 目录名前 3 字母（`gen_` / `prot_` / `hmi_` / `boot_`），不再用 `mod_`。
- 通过 `board_init()` 一次性把 Device 指针注入进模块（依赖注入，不在模块内部硬编码硬件）。
- 每个模块在 MANIFEST 声明它跑在哪个上下文（`fast` / `slow` / `hmi` / `main`），保证实时性纪律可见可查。

## 子目录（按上下文 + 前缀归位）

| 子目录 | prefix | ctx | 内容 |
|--------|--------|-----|------|
| **generate** | `gen_` | fast | 控制/生成——电机（MotApp/TurnCtrl/循迹/FCL）+ 功率（Buck/SuperCap/均流/SFRA/VSI/Balance/PowerCtrl） |
| **protect** | `prot_` | slow | 慢保护——状态聚合 / 心跳看门狗 / 去抖 / 软关断（`prot_monitor`） |
| **hmi** | `hmi_` | hmi | 通信 + 人机——帧协议 / 命令分发 / 串口/CAN/PMBus / 按键菜单（comm+hmi 合并） |
| **bootloader** | `boot_` | main | 升级流程：UART/CAN 升级 + CRC + 跳转 |

**ctx 值统一**：`generate→fast`、`protect→slow`、`hmi→hmi`、`bootloader→main`。

## 边界（别把什么放这里）

- ❌ 禁止直接碰寄存器 / HAL —— 一律调用 BSP 或 Devices
- ❌ 跨上下文传数据只用五原语（comp_io / 环形 / 邮箱 …），禁止裸共享变量
- 职责归属清晰：hmi（原 comm）只做传输，不做业务状态机；控制状态机归 generate

---

> 层级总览见 `../README.md`；跨上下文四档纪律见 `../docs/coding/concept.md`（本地根容器）。
