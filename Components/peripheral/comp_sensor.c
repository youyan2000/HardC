#include "comp_sensor.h"
#include <stddef.h>

// 基类构造 — 绑定名称, 清测量回调
void sensor_base_init(SensorBase *me, const char *name) {
  me->name = name;
  me->inited = 1;
  me->measure = NULL;
}

// 基类析构 — 清生命周期与测量回调
void sensor_base_deinit(SensorBase *me) {
  me->name = NULL;
  me->inited = 0;
  me->measure = NULL;
}

// 测量分发 — 未绑定 measure → 不支持
int sensor_measure(SensorBase *me) {
  if (me->measure != NULL) {
    return me->measure(me);
  }
  return ERR_NOT_SUPPORT;
}
