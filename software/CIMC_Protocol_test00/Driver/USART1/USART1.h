#ifndef __USART1_H
#define __USART1_H

#include "headerfiles.h"   // 包含你工程的基础头文件（gd32f10x.h 等）

/******************* RS485 控制引脚 *******************/
#define RS485_CS_PORT       GPIOA
#define RS485_CS_PIN        GPIO_PIN_1

#define RS485_TX_ENABLE()   gpio_bit_set(RS485_CS_PORT, RS485_CS_PIN)
#define RS485_RX_ENABLE()   gpio_bit_reset(RS485_CS_PORT, RS485_CS_PIN)

/* 接收缓冲区大小 */
#define RX_BUF_SIZE  512

/******************* 全局变量（供外部读取）*******************/
// 注意：.c 里相应变量不能加 static，否则这里会链接失败
extern uint8_t  recv_real_buf[RX_BUF_SIZE];
extern uint16_t recv_real_len;
extern uint8_t  recv_flag;
/******************* 函数声明 *******************/
void USART1_Config(void);                        // 初始化串口和485
void USART1_SendData(uint8_t *data, uint16_t len); // 阻塞发送，自动控制485方向
uint8_t* USART1_GetCommand(void);                // 获取接收到的命令帧
void USART1_ClearCommand(void);                  // 清除接收标志
void my_printf(const char *fmt, ...);            // 通过串口格式化打印

void usart_recv_task(void);                      // 接收处理任务，在main循环中调用
void USART1_IRQHandler(void);                    // 中断服务函数（通常不需要手动声明，保留也不影响）

#endif

