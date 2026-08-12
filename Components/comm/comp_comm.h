#ifndef COMP_COMM_H
#define COMP_COMM_H

// 通信平台层 —— 抽象基类
/*子类:
USART (drv_com_uart)
OLED (drv_com_oled)
KEY  (drv_com_key)
MPU6050 (drv_mpu6050) — I2C 通信类
*/
#include <stdint.h>
#include <stddef.h>

// 通信实例名
typedef enum {
  CommComputer,   // 蓝牙/上位机通信 (USART1)
  CommUltrasonic, // 超声波模块
  CommOled,       // OLED SSD1306 (软件 SPI)
  CommKey1,       // 按键 1 (PA11)
  CommKey2,       // 按键 2 (PB0)
  CommKey3,       // 按键 3 (PB12)
  CommKey4,       // 按键 4 (PB13)
  CommMpu6050     // MPU6050 六轴传感器 (软件 I2C)
} CommName;

typedef struct CommBase CommBase;

// 虚函数指针类型
typedef void    (*comm_s_fn)(CommBase *me, const uint8_t *dat, uint16_t len);  // 发送
typedef void    (*comm_b_fn)(CommBase *me);                                     // 开始接收
typedef uint8_t (*comm_r_fn)(CommBase *me);                                     // 读取

// 虚函数表 (ops)
typedef struct {
  comm_s_fn send;  // [必须] 发送数据
  comm_b_fn bgn;   // [必须] 启动中断接收
  comm_r_fn read;  // [必须] 读取一个字节 (按键读 GPIO, 其他返回 0 或 cur)
} CommOps;

// 基类结构体
struct CommBase {
  const CommOps *ops;  // 指向子类实现的虚函数表
  CommName name;       // 实例名
  uint8_t  *buf;       // 接收缓冲区指针
  uint8_t   cur;       // 当前收到的字节
};

// 基类构造 / 析构
void comm_base_init  (CommBase *me);
void comm_base_deinit(CommBase *me);

// 分发函数 —— 通过 ops 调用子类实现
void    comm_send(CommBase *me, const uint8_t *dat, uint16_t len);    // 发送
void    comm_bgn (CommBase *me);                                      // 开始接收
uint8_t comm_read(CommBase *me);                                      // 读取

#endif
