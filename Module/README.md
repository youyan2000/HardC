# Module — 本层是干什么的

> 层职责：**业务逻辑模块** —— 状态机 + 控制策略，组装 Components/Devices 成三件事。
> 层级模型 + 三上下文见 [concept](docs/concept.md)。

## 本层职责
- mod_*.h/c：一个模块 = 一件"事"（如 mod_buck / mod_supercap / mod_hmi / mod_sfra）。
- 通过指针注入绑定 Devices（board_init 里一次性接线）。
- 每个模块在 MANIFEST 声明 ctx: fast|slow|main（跑在哪个上下文）。

## 边界（别把什么放这里）
- 禁止直接碰寄存器 / HAL（调用 BSP 或 Devices）。
- 跨上下文传数据只用五原语，禁止裸共享变量（DESIGN-PRINCIPLES §3.4）。
- 业务逻辑归属明确：comm 瘦身、只做传输；控制状态机归 motor/power。

## 下层子目录
motor / power / comm / hmi — 见各 MANIFEST.yaml 的 ctx 与 description。
