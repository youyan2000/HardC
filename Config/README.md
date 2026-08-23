# Config — 本层是干什么的

> 层职责：**YAML 配置** —— 驱动的"零件清单 + 参数"。
> 装配线见 [concept](docs/concept.md) 与 [PLAN](./docs/PLAN.md)。

## 本层职责
- 	opologies/<topo>.yaml：拓扑**目录/模板**（声明用哪些模块、控制模块、参数表、status）。
- projects/<name>.yaml：具体**工程**（拓扑 + MCU + 名称实例化）。
- params/<name>_<variant>.yaml：参数变体（0xFB 槽位同源）。

## 边界（别把什么放这里）
- 只放配置，不放代码/脚本。
- 拓扑 status: ready 才可被 YmaC 生成工程；planned 仅为占位。
- 控制器/算法实现不在 Config（那是 Components/Devices/Module）。
