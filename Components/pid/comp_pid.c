// PID 平台层 —— 基类构造函数
// 极简: 只设默认时钟 10ms、输出不限幅、抗饱和默认关闭
// ops 由子类 init 时注入, 此处不绑定

#include "comp_pid.h"
#include <stddef.h>

void pid_base_init(PidBase *base) {
  base->ops         = NULL;
  base->dt          = 0.01f;
  base->out_min     = -1e9f;   // 默认无下限 (子类 init 应覆盖)
  base->out_max     =  1e9f;   // 默认无上限
  base->anti_windup = false;
}
