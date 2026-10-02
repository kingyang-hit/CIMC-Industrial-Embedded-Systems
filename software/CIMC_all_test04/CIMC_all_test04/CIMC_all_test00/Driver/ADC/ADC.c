/************************************************************
 * 版权：2025CIMC Copyright。 
 * 文件：adc.c
 * 作者: Lingyu Meng
 * 平台: 2025CIMC IHD-V04
 * 版本: Lingyu Meng     2025/03/07      V0.01    original
************************************************************/

/************************* 头文件 *************************/

#include "ADC.h"

/************************* 宏定义 *************************/

/************************ 变量定义 ************************/

/************************ 函数定义 ************************/

/************************************************************ 
 * Function :       ADC_Init
 * Comment  :       用于初始化ADC（不使用dma）
 * Parameter:       null
 * Return   :       null
 * Author   :       Lingyu Meng
 * Date     :       2025-02-30 V0.1 original
************************************************************/

void ADC_port_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOC);   // GPIOC时钟使能
	rcu_periph_clock_enable(RCU_ADC0);    // 使能ADC时钟
	
	gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0);   // 配置PC0为模拟输入
	gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1);
	
	adc_clock_config(ADC_ADCCK_PCLK2_DIV8);   // adc时钟配置
	

	
	

}

/************************************************************ 
 * Function :       ADC_Init
 * Comment  :       用于初始化ADC（不使用dma）
 * Parameter:       null
 * Return   :       null
 * Author   :       Lingyu Meng
 * Date     :       2025-02-30 V0.1 original
************************************************************/

void ADC_Init(void)
{
    adc_deinit();    // 复位ADC
	
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);    	// 使能连续转换模式
	   adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);    // 新增：启用扫描模式
	
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);   			// 数据右对齐 
    adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 2);  			// 通道配置，规则组0,1

    adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_10, ADC_SAMPLETIME_56);   // 对规则组进行配置
	  adc_routine_channel_config(ADC0, 1, ADC_CHANNEL_11, ADC_SAMPLETIME_56);
	
    adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, DISABLE);
	
    adc_enable(ADC0);   		// 使能ADC接口
	
    delay_1ms(1);  				// 等待1ms

    adc_calibration_enable(ADC0);    // ADC校准和复位ADC校准
	 adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL); //  规则采样软件触发
	
}


float ADC_Read_CH0_Voltage(void)
{
    // 连续扫描模式下，两个通道轮流转换，这里必须根据转换顺序读取
    // 方法：等待两个通道都转换完成，然后按顺序读取数据寄存器
    // 第1次读 -> 通道10的值，第2次读 -> 通道11的值
    while (adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET); // 等待第1个通道转换完成
    uint16_t ch0_raw = ADC_RDATA(ADC0);
    while (adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET); // 等待第2个通道转换完成
    uint16_t ch1_raw = ADC_RDATA(ADC0); // 此处不使用，仅用于清空

    return ch0_raw * 3.3f / 4095.0f;
}

float ADC_Read_DAC_Readback_Voltage(void)
{
    // 同理，但返回的是通道11的值
    while (adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET);
    uint16_t ch0_raw = ADC_RDATA(ADC0); // 丢弃
    while (adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET);
    uint16_t ch1_raw = ADC_RDATA(ADC0);

    return ch1_raw * 3.3f / 4095.0f;
}
/****************************End*****************************/

