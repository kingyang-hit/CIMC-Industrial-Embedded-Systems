/************************************************************
 * 版权：2025CIMC Copyright。
 * 文件：lowpower.c
 * 作者: Jialei Zhao
 * 平台: 2025CIMC IHD-V04
 * 版本: Jialei Zhao     2026/06/01     V0.01    original
************************************************************/

#include "PMU.h"

/************************* 宏定义 *************************/

#define RCU_MODIFY_4(__delay)   do{                                     \
                                    volatile uint32_t i, reg;           \
                                    if(0 != __delay){                   \
                                        /* Insert a software delay */   \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        reg = RCU_CFG0;                 \
                                        reg &= ~(RCU_CFG0_AHBPSC);      \
                                        reg |= RCU_AHB_CKSYS_DIV2;      \
                                        /* AHB = SYSCLK/2 */            \
                                        RCU_CFG0 = reg;                 \
                                        /* Insert a software delay */   \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        reg = RCU_CFG0;                 \
                                        reg &= ~(RCU_CFG0_AHBPSC);      \
                                        reg |= RCU_AHB_CKSYS_DIV4;      \
                                        /* AHB = SYSCLK/4 */            \
                                        RCU_CFG0 = reg;                 \
                                        /* Insert a software delay */   \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        reg = RCU_CFG0;                 \
                                        reg &= ~(RCU_CFG0_AHBPSC);      \
                                        reg |= RCU_AHB_CKSYS_DIV8;      \
                                        /* AHB = SYSCLK/8 */            \
                                        RCU_CFG0 = reg;                 \
                                        /* Insert a software delay */   \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        reg = RCU_CFG0;                 \
                                        reg &= ~(RCU_CFG0_AHBPSC);      \
                                        reg |= RCU_AHB_CKSYS_DIV16;     \
                                        /* AHB = SYSCLK/16 */           \
                                        RCU_CFG0 = reg;                 \
                                        /* Insert a software delay */   \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                    }                                   \
                                }while(0)

/************************************************************
 * Function :       lowpower_init
 * Comment  :       低功耗初始化，配置唤醒源
 * Parameter:       null
 * Return   :       null
************************************************************/
void lowpower_init(void)//配个按键，NB
{
    
	rcu_periph_clock_enable(RCU_SYSCFG);    // 初始化SYSCFG时钟
	rcu_periph_clock_enable(RCU_GPIOE);    // 初始化GPIO_E总线时钟

	//初始化按键端口
	gpio_mode_set(GPIOE , GPIO_MODE_INPUT , GPIO_PUPD_PULLUP , GPIO_PIN_6 );   			// GPIO模式设置为输入，上拉	

	//!配置中断
	nvic_irq_enable(EXTI5_9_IRQn , 2U , 0U);
	/* connect key EXTI line to key GPIO pin */
	syscfg_exti_line_config(EXTI_SOURCE_GPIOE , EXTI_SOURCE_PIN6);

	/* configure key EXTI line */
	exti_init(EXTI_6 , EXTI_INTERRUPT , EXTI_TRIG_FALLING);
	exti_interrupt_flag_clear(EXTI_6);
	 exti_interrupt_enable(EXTI_6);
	pmu_wakeup_pin_enable();
	
}
// PE6外部中断处理（KEY5(PIN6)唤醒）
void EXTI5_9_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_6)) {
        exti_interrupt_flag_clear(EXTI_6);
			  
    }
}


/************************************************************
 * Function :       lowpower_sleep
 * Comment  :       睡眠模式，任意中断唤醒
 * Parameter:       null
 * Return   :       null
************************************************************/
void lowpower_sleep(void)
{
	  rcu_periph_clock_enable(RCU_PMU);
    my_printf("process enter sleep mode ....\r\n");
	   SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    pmu_to_sleepmode(WFI_CMD);
	  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	   delay_1ms(100);
    
	  
}

/************************************************************
 * Function :       lowpower_deepsleep
 * Comment  :       深度睡眠模式，KEY1或串口唤醒
 * Parameter:       null
 * Return   :       null
************************************************************/
void lowpower_deepsleep(void)
{

    rcu_periph_clock_enable(RCU_PMU);
    
    // 清除残留的 EXTI 17 中断标志，防止刚进睡眠就立刻被误唤醒
    exti_interrupt_flag_clear(EXTI_17); 
    
    usart_mute_mode_enable(USART1);
    
    delay_1ms(50);  // 等待串口打印完成
    
    // 进入深度睡眠模式
    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, PMU_LOWDRIVER_ENABLE, WFI_CMD);
    
    // 唤醒后恢复外设
    //rcu_config();       // 重新配置系统时钟
    //USART1_Config();    // 重新初始化串口
    //usart_mute_mode_disable(USART1);
    //delay_1ms(100);
     rcu_config();       
    USART1_Config();    
    usart_mute_mode_disable(USART1);
    delay_1ms(100);

}

/************************************************************
 * Function :       lowpower_standby
 * Comment  :       待机模式，WKUP引脚唤醒后复位/RTC宏定义，4s唤醒
 * Parameter:       null
 * Return   :       null
************************************************************/

/************************************************************
 * Function :       _soft_delay_
 * Comment  :       软件延时函数
 * Parameter:       time: 延时时间，单位：ms
 * Return   :       null
 * Author   :       Jialei Zhao
 * Date     :       2026-01-30 V0.1 original
************************************************************/
static void _soft_delay_(uint32_t time)
{
    __IO uint32_t i;
    for (i = 0; i < time * 10; i++)
    {
    }
}
/************************************************************
 * Function :       rcu_config
 * Comment  :       用于配置时钟到240MHZ
 * Parameter:       null
 * Return   :       null
 * Author   :      	Jialei Zhao
 * Date     :       2026-01-30 V0.1 original
************************************************************/
static void rcu_config(void)
{
	uint32_t timeout = 0U;
	uint32_t stab_flag = 0U;

	//!时钟切换demo里面的
	/* It is strongly recommended to include it to avoid issues caused by self-removal. */
	RCU_MODIFY_4(0x50);
	/* select HXTAL as system clock source, deinitialize the RCU */
	rcu_system_clock_source_config(RCU_CKSYSSRC_HXTAL);
	/* It is strongly recommended to include it to avoid issues caused by self-removal. */
	_soft_delay_(200);
	rcu_deinit();

	/* enable HXTAL */
	RCU_CTL |= RCU_CTL_HXTALEN;

	/* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
	do
	{
		timeout++;
		stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
	} while ((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

	/* if fail */
	if (0U == (RCU_CTL & RCU_CTL_HXTALSTB))
	{
		while (0U == (RCU_CTL & RCU_CTL_HXTALSTB))
		{
		}
	}

	RCU_APB1EN |= RCU_APB1EN_PMUEN;
	PMU_CTL |= PMU_CTL_LDOVS;

	/* HXTAL is stable */
	/* AHB = SYSCLK */
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	/* APB2 = AHB/2 */
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV2;
	/* APB1 = AHB/4 */
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV4;

	/* Configure the main PLL, PSC = 25, PLL_N = 480, PLL_P = 2, PLL_Q = 10 */
	RCU_PLL = (25U | (480U << 6U) | (((2U >> 1U) - 1U) << 16U) |
		(RCU_PLLSRC_HXTAL) | (10U << 24U));

	/* enable PLL */
	RCU_CTL |= RCU_CTL_PLLEN;

	/* wait until PLL is stable */
	while (0U == (RCU_CTL & RCU_CTL_PLLSTB))
	{
	}

	/* Enable the high-drive to extend the clock frequency to 240 Mhz */
	PMU_CTL |= PMU_CTL_HDEN;
	while (0U == (PMU_CS & PMU_CS_HDRF))
	{
	}

	/* select the high-drive mode */
	PMU_CTL |= PMU_CTL_HDS;
	while (0U == (PMU_CS & PMU_CS_HDSRF))
	{
	}

	/* select PLL as system clock */
	RCU_CFG0 &= ~RCU_CFG0_SCS;
	RCU_CFG0 |= RCU_CKSYSSRC_PLLP;

	/* wait until PLL is selected as system clock */
	while (0U == (RCU_CFG0 & RCU_SCSS_PLLP))
	{
	}
}


/****************************End*****************************/









