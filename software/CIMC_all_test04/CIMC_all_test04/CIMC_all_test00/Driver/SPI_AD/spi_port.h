/**
 * @file spi_port.h
 * @brief SPI硬件抽象层接口定义
 * @version 2026-04-25
 */

#ifndef SPI_PORT_H // 
#define SPI_PORT_H

#include "stdint.h" // 确保 uint16_t 定义可用

// ==================================================
// 用户配置区 (请根据你的硬件原理图修改)
// ==================================================

// 1. 定义 SPI 片选 (CS) 引脚操作

#define AD3344_CS_PORT    GPIOA
#define AD3344_CS_PIN     GPIO_PIN_4
#define  SPI_RESET_CS_LOW()            gpio_bit_reset(GPIOA, GPIO_PIN_4)
#define  SPI_SET_CS_HIGH()           gpio_bit_set(GPIOA, GPIO_PIN_4)
// 2. 宏定义：SPI 片选控制
// 如果你的工程中没有定义 GPIO_BIT_SET/RESET，请使用标准库函数
#ifndef SPI_CLR_CS
    #define SPI_CLR_CS()  gpio_bit_reset(AD3344_CS_PORT, AD3344_CS_PIN)
#endif

#ifndef SPI_SET_CS
    #define SPI_SET_CS()  gpio_bit_set(AD3344_CS_PORT, AD3344_CS_PIN)
#endif

// ==================================================
// 函数声明
// ==================================================

/**
 * @brief 初始化 SPI 硬件端口
 */
void ad3344_spi_port_init(void);

/**
 * @brief 16位 SPI 发送接收函数
 * @param tx_data 待发送的数据 (16位)
 * @return uint16_t 接收到的数据
 */
uint16_t ad3344_spi_txrx16bit(uint16_t tx_data);

#endif // SPI_PORT_H

