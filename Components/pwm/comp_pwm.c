// PWM 平台层 —— 基类构造函数
// 极简: 默认频率 100kHz、占空比不限幅、ops 由子类 init 时注入

#include "comp_pwm.h"
#include <stddef.h>

void pwm_base_init(PwmBase *me) {
  me->ops      = NULL;
  me->mode     = PwmMode_Buck;
  me->num_ch   = 1;
  me->freq_hz  = 100000;    // 默认 100kHz
  me->duty_min = 0.0f;      // 子类 init 应覆盖为合理物理约束
  me->duty_max = 1.0f;
  me->running  = false;
}
