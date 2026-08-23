# Devices — 本层是干什么的

> 层职责：**具体硬件/算法子类实现**。继承父类，绑到某颗芯片或某个具体算法变体。
> 层级模型见 [concept](docs/concept.md)。

## 本层职责
- 一个子类一个文件（如 pid_standard / pwm_svpwm / adc_dc_sampler）。
- 继承 Components 的父类（父类为第一成员 + container_of 下溯 + 构造器绑定 ops）。
- 按域归子目录（adc/comm/pid/pll/pwm/motor/peripheral），含 MANIFEST.yaml。

## 边界（别把什么放这里）
- 依赖 BSP 接口，禁止绕过 BSP 直接滚裸寄存器（硬件差异归 BSP）。
- 不做业务逻辑 / 状态机（那是 Module 层的责任）。
- 不做平台无关的纯算法（那属于 Components）。

## 下层子目录
见各子目录 MANIFEST.yaml 的 description。
