# App — 本层是干什么的

> 层职责：**应用入口 + 三上下文接线**。只有一组文件，禁止自由发挥。
> 层级模型 + 三上下文见 [concept](docs/concept.md)。

## 本层职责
- pp_main.c/h：根结构体 ProjectRoot（值包含全实例）、配置 POD ProjectConfig、board_init（唯一绑定点）、apply_config、三上下文 ISR 钩子、BackgroundTask。
- 模板：App/app_main.c.tmpl + app_main.h.tmpl（YmaC 物化目标）。

## 边界（别把什么放这里）
- App 层只有一组文件，基于模板开始，禁止自由发挥（CLAUDE.md）。
- 不做业务逻辑（那些在 Module）；App 只排时序和接线。
- 三个上下文纪律：FAST 禁 printf/I2C/OLED/malloc；MAIN 做所有慢 I/O。

## 相关
- 调参协议：App/pid_tune.h/c（0xFB 帧）。
