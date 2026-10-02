/************************************************************
 * 文件：USART1.c
 * 说明：USART1 + RS485 简易驱动（无DMA版本）
 *       接收用中断+空闲检测，发送用阻塞方式
 ************************************************************/

#include "USART1.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "LED.h"

/************************* 宏定义 *************************/
#define RX_BUF_SIZE   512

// RS485 片选引脚（请根据实际硬件修改）
#define RS485_CS_PORT  GPIOA
#define RS485_CS_PIN   GPIO_PIN_1

#define RS485_TX_ENABLE()  gpio_bit_set(RS485_CS_PORT, RS485_CS_PIN)
#define RS485_RX_ENABLE()  gpio_bit_reset(RS485_CS_PORT, RS485_CS_PIN)

/************************ 全局变量 ************************/
uint8_t recv_real_buf[RX_BUF_SIZE];
uint16_t recv_index = 0;        // 当前写入位置
uint16_t recv_real_len = 0;            // 一帧长度
uint8_t recv_flag = 0;                // 一帧完成标志
extern uint8_t  g_baudrate_index;
/************************ 辅助函数 ************************/
static void rs485_set_mode(uint8_t tx_mode) {
    if (tx_mode) {
        RS485_TX_ENABLE();
    } else {
        RS485_RX_ENABLE();
    }
}

/************************************************************
 * 功能：初始化 USART1、GPIO 和 RS485 控制引脚
 ************************************************************/
void USART1_Config(void) {
    // 时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART1);

    // TX：PA2，AF 推挽输出
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_2);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    // RX：PA3，AF 输入
    gpio_af_set(GPIOA, GPIO_AF_7, GPIO_PIN_3);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_3);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);

    // RS485 CS：PA1，推挽输出
    gpio_mode_set(RS485_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, RS485_CS_PIN);
    gpio_output_options_set(RS485_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, RS485_CS_PIN);
    rs485_set_mode(0);   // 默认接收

    // USART 参数
    usart_deinit(USART1);
    usart_word_length_set(USART1, USART_WL_8BIT);
    usart_stop_bit_set(USART1, USART_STB_1BIT);
    usart_parity_config(USART1, USART_PM_NONE);
    if(g_baudrate_index == 17){
		usart_baudrate_set(USART1, 4800U);
		}
		else if(g_baudrate_index == 18){
			usart_baudrate_set(USART1, 9600U);}
				else if(g_baudrate_index == 19){
			usart_baudrate_set(USART1, 19200U);}
			else if(g_baudrate_index == 20){
			usart_baudrate_set(USART1, 115200U);}
			else {
		usart_baudrate_set(USART1, 115200U);}
		//usart_baudrate_set(USART1, 115200U);
    usart_receive_config(USART1, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
    usart_enable(USART1);

    // 中断：接收非空 + 空闲中断
    nvic_irq_enable(USART1_IRQn, 5, 0);
    usart_interrupt_enable(USART1, USART_INT_RBNE);
    usart_interrupt_enable(USART1, USART_INT_IDLE);

    // 初始化缓冲区
    memset(recv_real_buf, 0, sizeof(recv_real_buf));
    recv_index = 0;
    recv_real_len = 0;
    recv_flag = 0;
}

/************************************************************
 * 功能：阻塞发送，自动切换 485 方向
 ************************************************************/
void USART1_SendData(uint8_t *data, uint16_t len) {
    if (len == 0 || data == NULL) return;

    rs485_set_mode(1);   // 发送

    for (uint16_t i = 0; i < len; i++) {
        usart_data_transmit(USART1, data[i]);
        while (usart_flag_get(USART1, USART_FLAG_TBE) == RESET);
    }

    // 等待最后一个字节真正发完
    while (usart_flag_get(USART1, USART_FLAG_TC) == RESET);

    rs485_set_mode(0);   // 切回接收
}

/************************************************************
 * 功能：中断处理（接收字节 + 帧结束判断）
 ************************************************************/
void USART1_IRQHandler(void) {
    // ---- 接收非空 ----
    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE) != RESET) {
        uint8_t data = usart_data_receive(USART1);
        if (recv_index < RX_BUF_SIZE) {
            recv_real_buf[recv_index++] = data;
        } else {
            // 溢出处理，可丢弃或清空
            recv_index = 0;
        }
    }

    // ---- 空闲中断：一帧接收完毕 ----
    if (usart_interrupt_flag_get(USART1, USART_INT_FLAG_IDLE) != RESET) {
        usart_data_receive(USART1);  // 清除空闲标志

        if (recv_index > 0) {
            recv_real_len = recv_index;
            recv_flag = 1;
        }
        recv_index = 0;  // 准备下一帧
    }
}

/************************************************************
 * 功能：获取接收到的命令（一帧）
 ************************************************************/
uint8_t* USART1_GetCommand(void) {
    static uint8_t cmd_buf[128];
    if (recv_flag) {
        uint16_t len = recv_real_len < sizeof(cmd_buf) - 1 ?
                       recv_real_len : sizeof(cmd_buf) - 1;
        memcpy(cmd_buf, recv_real_buf, len);
        cmd_buf[len] = '\0';
        return cmd_buf;
    }
    return NULL;
}

/************************************************************
 * 功能：清除接收标志，释放缓冲区（外部调用）
 ************************************************************/
void USART1_ClearCommand(void) {
    recv_flag = 0;
    recv_real_len = 0;
    // 不清缓冲区内容，仅重置索引
}

/************************************************************
 * 功能：简易 printf 通过 485 输出
 ************************************************************/
void my_printf(const char *fmt, ...) {
    static char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    USART1_SendData((uint8_t*)buf, strlen(buf));
}

/************************************************************
 * 功能：在主循环中调用的任务，处理接收并打印
 ************************************************************/
void usart_recv_task(void) {
	  
    
    if (recv_flag) {
        // 判断是否为 RTC 相关指令（避免把 "RTC Config" 打印成普通数据）

        my_printf("Recv [%d]: %s\r\n", recv_real_len, recv_real_buf);
        
        USART1_ClearCommand();
    }
}