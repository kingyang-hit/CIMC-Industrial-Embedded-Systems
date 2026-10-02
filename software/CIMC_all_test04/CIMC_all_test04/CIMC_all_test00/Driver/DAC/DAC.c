#include "DAC.h"


void dac_config(void)
{
	
	   /* 使能时钟*/
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_DAC);
	  //DAC0 PA4通道0，  PA5通道1   输出引脚配置
    /* configure PA4 as DAC output */
    gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_4);
    /* initialize DAC */
    dac_deinit(DAC0);
    /* DAC trigger disable */
    dac_trigger_disable(DAC0, DAC_OUT0);
    /* DAC wave mode config */
    dac_wave_mode_config(DAC0, DAC_OUT0, DAC_WAVE_DISABLE);
    
    /* DAC output buffer enable */
    dac_output_buffer_enable(DAC0, DAC_OUT0);
    /* DAC enable */
    dac_enable(DAC0, DAC_OUT0);
	
    
}

