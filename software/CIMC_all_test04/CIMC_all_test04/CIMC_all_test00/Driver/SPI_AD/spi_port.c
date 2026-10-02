/**
 * @file spi_port.c
 * @brief SPI底层硬件驱动实现
 * @version 2026-04-25
 */

#include "gd32f4xx.h" // 请根据你的具体GD32型号调整
#include "spi_port.h"
#include "gd30ad3344.h" // 包含头文件以获取配置

// ================= 硬件配置区 (请根据原理图修改) =================
#define AD3344_SPI_PORT     GPIOA
#define AD3344_SPI          SPI0
#define AD3344_CS_PIN       GPIO_PIN_4
#define AD3344_SCK_PIN      GPIO_PIN_5
#define AD3344_MISO_PIN     GPIO_PIN_6 
#define AD3344_MOSI_PIN     GPIO_PIN_7

/**
 * @brief SPI端口初始化
 * @note 修正了原代码中可能存在的时钟配置问题，使用16位模式
 */
void ad3344_spi_port_init(void) {
    spi_parameter_struct spi_init_struct;

    // 1. 开启时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);

    // 2. 配置CS引脚 (GPIO控制)  4
	  gpio_mode_set(AD3344_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,  AD3344_CS_PIN);
    gpio_output_options_set(AD3344_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, AD3344_CS_PIN);
    SPI_SET_CS_HIGH(); // 默认拉高，空闲状态

    // 3. 配置SPI复用引脚 (SCK, MISO, MOSI)
    gpio_af_set(AD3344_SPI_PORT , GPIO_AF_5, GPIO_PIN_5|GPIO_PIN_6| GPIO_PIN_7);
    gpio_mode_set(AD3344_SPI_PORT , GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5|GPIO_PIN_6| GPIO_PIN_7);
    gpio_output_options_set(AD3344_SPI_PORT , GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_5|GPIO_PIN_6| GPIO_PIN_7);

    // 4. SPI 参数配置 (参考数据手册，AD3344通常使用Mode 1)
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode          = SPI_TRANSMODE_FULLDUPLEX; // 全双工
    spi_init_struct.device_mode         = SPI_MASTER;               // 主机模式
    spi_init_struct.frame_size          = SPI_FRAMESIZE_16BIT;      // 关键：16位帧
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE; // Mode 1 (CPOL=0, CPHA=1)
    spi_init_struct.nss                 = SPI_NSS_SOFT;            // 软件管理NSS
    spi_init_struct.prescale            = SPI_PSC_64;             // 分频系数 (约几百kHz)
    spi_init_struct.endian              = SPI_ENDIAN_MSB;           // 高位在前

    spi_init(AD3344_SPI, &spi_init_struct);
    spi_enable(AD3344_SPI);
}

/**
 * @brief 16位SPI收发函数 (修正版)
 * @param tx_data 发送的数据
 * @return uint16_t 接收到的数据
 * @note 原代码中此函数缺失，导致编译无法通过
 */
uint16_t ad3344_spi_txrx16bit(uint16_t tx_data) {
    // 等待发送缓冲区空
    while(spi_i2s_flag_get(AD3344_SPI, SPI_FLAG_TBE) == RESET);
    spi_i2s_data_transmit(AD3344_SPI, tx_data);

    // 等待接收缓冲区非空
    while(spi_i2s_flag_get(AD3344_SPI, SPI_FLAG_RBNE) == RESET);
    
    return (uint16_t)spi_i2s_data_receive(AD3344_SPI);
}

/**
 * @brief 利用 SCLK 拉低 28ms 特性，强制复位 AD3344 的 SPI 接口
 */
void AD3344_Hardware_ResetViaSCLK(void)
{
    // 1. 确保当前 CS 为高，防止 ADC 误判通信
    SPI_SET_CS_HIGH(); 
    
    // 2. 将 SCK 引脚 (PA5) 配置为普通 GPIO 输出
    gpio_mode_set(AD3344_SPI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, AD3344_SCK_PIN);
    gpio_output_options_set(AD3344_SPI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, AD3344_SCK_PIN);
    
    // 3. 强行拉低 SCK
    gpio_bit_reset(AD3344_SPI_PORT, AD3344_SCK_PIN);
    
    // 4. 保持低电平 50ms (远超数据手册要求的 28ms)
    delay_1ms(50); 
    
    // 5. 恢复 SCK 引脚为 SPI 复用功能，并重新初始化 SPI 外设
    ad3344_spi_port_init(); 
    
    delay_1ms(10); // 等待 ADC 内部复位就绪
}