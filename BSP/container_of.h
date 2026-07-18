#ifndef CONTAINER_OF_H
#define CONTAINER_OF_H

// container_of —— 从成员指针反推父结构体指针
// 用法: container_of(base_ptr, DerivedType, base_member_name)
// 原理: (成员地址) - (成员在结构体中的偏移) = 结构体首地址
// 这是 Linux 内核经典宏，实现 C 语言面向对象中的"向下转型"

#include <stddef.h>

#define container_of(ptr, type, member) \
  ((type *)((char *)(ptr) - offsetof(type, member)))

#endif
