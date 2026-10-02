/*!
    \file    gd30ad3344.c
    \brief   gd30ad3344 driver
    
    \version 2024-10-08, V1.0.0, firmware for GD30AD3344
*/

#include "gd30ad3344.h"
#include "spi_port.h"


uint16_t ADC_Config[2]={0}; 
uint16_t AD3344_CONFIG;

/*!
    \brief      delay us
    \param[in]  t: delay time
    \param[out] none
    \retval     none
*/
void delay_us(uint32_t t)
{
    uint16_t i;
    while (t--){
         i = 10;
         while(i--);
   }
}

/*!
    \brief      exti-line enable (PA6)
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ad3344_Exit_enable(void)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable( RCU_SYSCFG);
    
     gpio_mode_set(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    /* connect key wakeup EXTI line to key GPIO pin */
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN6);
    /* configure key wakeup EXTI line */
    exti_init(EXTI_6, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_6);
    
    nvic_irq_enable(EXTI5_9_IRQn, 2U, 0U);
}

/*!
    \brief      exti-line disable
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ad3344_Exit_disable(void)
{
    nvic_irq_disable(EXTI5_9_IRQn);
    exti_interrupt_flag_clear(EXTI_6);
    exti_interrupt_disable(EXTI_6);
    
    rcu_periph_clock_enable(RCU_SPI0);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
	 gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
}

/*!
    \brief      GD30AD3344 transmit data
    \param[in]  config_d: register value
    \param[out] none
    \retval     the read value of register
*/
uint16_t AD3344_Send_Data(uint16_t config_d)
{
    uint16_t Data;

    Data = ad3344_spi_txrx16bit(config_d);
    
    return (Data);
}

/*!
    \brief      GD30AD3344 Config Register(32bit trans)
    \param[in]  config_d: the data need to be tramit
    \param[in]  *config: Register readback value
    \param[out] none
    \retval     the read value of register
*/
uint16_t ad3344_read_data32(uint16_t config_d, uint16_t *config)
{
    uint16_t data;
    
    data = AD3344_Send_Data(config_d);
    *config = AD3344_Send_Data(0);
    
    return (data);
}

/*!
    \brief      GD30AD3344 Config Register(16bit trans)
    \param[in]  config_d: the data need to be tramit
    \param[out] none
    \retval     the read value of register
*/
uint16_t ad3344_read_data16(uint16_t config_d)
{
    uint16_t data;
    
    SPI_CLR_CS();
    //delay_us(1000);
    
    data = AD3344_Send_Data(config_d);
    delay_us(10);
    SPI_SET_CS();
    //delay_us(10000);
	  
    
    return (data);
}

/*!
    \brief      GD30AD3344 Read Register
    \param[in]  addr
      \arg      0x01: Config Register
    \param[out] none
    \retval     the read value of register
*/
uint16_t ad3344_read_regs(uint8_t addr)
{
    uint8_t reg_addr = addr;
    uint16_t reg_rtu = 0;

    SPI_CLR_CS();
    delay_us(1000);
    
    AD3344_Send_Data(reg_addr);
    
    reg_rtu = AD3344_Send_Data(0x00);
    delay_us(10);
    SPI_SET_CS();
    //delay_us(10000);



    return reg_rtu;
}

/*!
    \brief      GD30AD3344 Init
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ad3344_init(uint16_t config_d)
{
    SPI_CLR_CS();
    //delay_us(1000);
	   delay_us(10);
    
    #ifdef BIT32_TRANS_CYCLE
    ad3344_read_data32(config_d, ADC_Config);
    #else
    ad3344_spi_txrx16bit(config_d);
    #endif
    
    delay_us(100);
    
    SPI_SET_CS();
    delay_us(1000);


}

/*!
    \brief      GD30AD3344 stop conversion
    \param[in]  none
    \param[out] none
    \retval     none
*/
void ad3344_stop_conver()
{
    AD3344_CONFIG |= AD3344_REG_CONFIG_MODE_SINGLE;
	  
    
    #ifdef BIT32_TRANS_CYCLE
    ad3344_read_data32(AD3344_CONFIG, ADC_Config);
    #else
    ad3344_spi_txrx16bit(AD3344_CONFIG);
    #endif
}

/*!
    \brief      GD30AD3344 reset
    \param[in]  none
    \param[out] none
    \retval     the result of the conversion
*/
void ad3344_reset()
{
    #ifdef BIT32_TRANS_CYCLE
    ad3344_read_data32(AD3344_CONFIG_DEFAULT, ADC_Config);
    #else
    ad3344_spi_txrx16bit(AD3344_CONFIG_DEFAULT);
    #endif
}
// ================= 【新增】实用读取函数 =================
/**
 * @brief 读取ADC转换结果 (推荐使用此函数)
 * @return int16_t 转换结果 (有符号数)
 */
int16_t ad3344_read_adc(void) {
    uint16_t raw_data;
    
    SPI_CLR_CS();
    raw_data = AD3344_Send_Data(AD3344_CONVERSION_ADDRESS); // 读取转换寄存器
    SPI_SET_CS();
    
    // 将16位无符号数转换为有符号数 (二进制补码)
    // 因为AD3344输出是二进制补码格式
  return (int16_t)raw_data;
}
