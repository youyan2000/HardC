# Components — 本层是干什么的

> 层职责：**父类 + 契约 + 纯算法**。定义接口（ops 虚表），不绑定芯片。
> 层级模型见 [concept](docs/concept.md)。

## 本层职责
- 定义接口**契约**：父类 + ops 虚表（PidBase / PwmBase / AdcBase / CommBase / PowerStage / PllBase…）。
- 放**平台无关的纯算法组件**（comp_dlog / comp_pfc / comp_crc / comp_viterbi / comp_rs / comp_math…）。
- 每个子目录一个域（adc/comm/pid/pll/pwm/motor/power/protection/math/codec/contract/peripheral/dsp/database），含 MANIFEST.yaml。

## 边界（别把什么放这里）
- **禁止 include HAL / Driverlib / 平台头** —— 硬件走 BSP 不透明句柄（DESIGN-PRINCIPLES §七）。
- 不放具体芯片实现（那是 Devices 层）。
- 不放业务状态机（那是 Module 层）。
- 纯算法组件保持 -Wall -Wextra -Werror 零警告、可 host 单测。

## 下层子目录
见各子目录 MANIFEST.yaml 的 description。
