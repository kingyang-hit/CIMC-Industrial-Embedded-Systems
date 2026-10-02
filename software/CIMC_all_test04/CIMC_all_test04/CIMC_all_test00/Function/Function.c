
/************************************************************
 * 版权：2025CIMC Copyright。 
 * 文件：Function.c
 * 作者: Lingyu Meng
 * 平台: 2025CIMC IHD-V04
 * 版本: Lingyu Meng     2025/2/16     V0.01    original
************************************************************/

#include "Function.h"


//宏定义
#define ALARM_LOG_MAX 10
#define APP_CONFIG_SECTOR_SIZE 4096
// ========== 全局变量 ==========

extern uint32_t  current_baudrate; //波特率
extern float    ch0_ratio;
extern float    ch1_ratio;
extern float    ch0_thresh;
extern float    ch1_thresh;
static uint8_t auto_report_enabled = 0;      // 自动上报使能标志

static uint8_t auto_report_interval = 1;     // 上报间隔：1=1s, 2=3s, 3=5s
static uint32_t last_report_time = 0;        // 上次上报时间（ms）
extern uint32_t log_write_addr;   // 当前写入指针

extern uint8_t  g_baudrate_index; //波特率索引
// 全局写入指针
uint32_t log_write_addr = ALARM_LOG_START_ADDR; 
volatile uint8_t need_reset_for_baudrate = 0;  //重启标志位

uint16_t i = 0, count, result = 0;
float limit_data=1.0;
int major;
float mini;
 sample_interval_t cur_interval = SAMPLE_5S;   // 当前周期，默认5秒
//ADC相关变量
int adc0_value;  // ADC0采样值
int adc1_value; 
float Vol_Value0;  // ADC0采样值转换成电压值
float Vol_Value1;
//AD3344相关变量

float VREF = 3.3f ;
int TABLE_SIZE = 12;
float pt100_table[12][2] = { {-49.27,80.6}, {-44.49,82.5},{0,100.0}, {20,107.79},{33.44,113}, {38.61,115},{40,115.54}, 
{60,123.24},{80,130.90}, {100,138.51},{130.45,150}, {141.11,154}}; // 完整表
uint8_t pt = 0;
extern rtc_parameter_struct   rtc_initpara;
		//zhenyongde
		uint8_t *rx_buf;
		uint16_t rx_len;
    uint8_t bin_frame[256];
    uint16_t bin_len = 0;
		uint16_t g_device_id = 0x0001;    // 全局设备ID，默认 0x0001
		uint32_t g_firmware = 0x02000100;  // 当前固件版本
		uint32_t g_device_time = 0x6A09A79F;  // 当前时间戳（初始值示例）
		typedef union {
    float    f;
    uint32_t u;
    uint8_t  b[4];      // 便于逐字节存取（大端序需注意字节序）
} Float_IEEE754;

		float g_ch0_ratio = 1.0f;   // CH0 变比，默认 1.0
		float g_ch1_ratio = 1.0f;   // CH1 变比，默认 1.0
		float g_pt100_ratio = 1.0f;   // CH2 变比，默认 1.0
		uint8_t g_report_interval = 1;   // 上报间隔索引：1=1s, 2=3s, 3=5s
		uint8_t g_report_enable = 0;     // 0=停止上报, 1=定时上报中
		static uint32_t last_report_tick = 0;
		volatile uint32_t sys_tick = 0;   // 由 SysTick_Handler 每 1ms 加 1
		float g_yuzhi_ch0 = 21.59f;   // 默认阈值示例
		float g_yuzhi_ch1 = 21.59f;
		float g_yuzhi_pt = 21.59f;   // 若有 CH2
		uint8_t  resp_type = FRAME_TYPE_RESP;
		uint8_t g_alarm_enable = 02;     // 01=主动上报, 02=不主动上报
		uint8_t g_chaxun_enable = 0;     // 1=查询报警记录 0=不查询报警记录
		uint8_t alarm_clear = 0;     // 1=清除报警记录 0=不清除报警记录		
    float dac_voltage;
		uint8_t sleep_sign = 0;
		uint8_t bote = 0;
int16_t adc_raw;
extern volatile uint32_t system_tick;
/************************ 提前声明各个命令的处理函数***********************/
static void cmd_handler_reset(void);
static void cmd_handler_rtc_config(void);
static void cmd_handler_rtc_now(void);
static void cmd_handler_set_id(void);
static void cmd_handler_read_id(void);
static void cmd_handler_set_brate(void);
static void cmd_handler_read_brate(void);
static void cmd_handler_read_CH0(void);
static void cmd_handler_read_CH1(void);
static void cmd_handler_read_limit(void);
static void cmd_handler_set_ch0_ratio(void);
static void cmd_handler_set_ch1_ratio(void);
static void cmd_handler_set_ch0_thresh(void);
static void cmd_handler_set_ch1_thresh(void);
static void cmd_handler_start(void);
static void cmd_handler_stop(void);
static void cmd_handler_deepsleep(void);
static void cmd_handler_config_save(void);
static void cmd_handler_config_read(void);
static void cmd_handler_hide(void);
static void cmd_handler_unhide(void);
void get_current_time(Alarm_Log_TypeDef* log);
static void send_error_frame(void);//错误帧
static void send_heartbeat(void);//心跳帧
 // 设置升级标志并复位
static void set_upgrade_flag_and_reset(void);
void set_upgrade_flag(void);
float pt100_lookup_temp(float R);
/************************************************************ 
 * Function : System_Init
 * Comment  : 用于初始化MCU
 * Parameter: null
 * Return   : null
 * Author   : Lingyu Meng
 * Date     : 2025-02-30 V0.1 original
************************************************************/
void System_Init(void)
{
    systick_config();     // 时钟配置
			// 重映射中断向量表到 App 起始地址 0x08011000
	SCB->VTOR = FLASH_BASE | 0x11000;  
	// 必须加的指令，保证配置立即生效
	__DSB();
	__ISB();
	 __enable_irq();
	  rcu_periph_clock_enable(RCU_GPIOC);   // GPIOA时钟使能
	  gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_0);   // GPIO 模式设置
	  ADC_port_init();
	  ADC_Init();  // 内部ADC0配置
	  dac_config(); //DAC配置
	  //pmu初始化
	  rcu_periph_clock_enable(RCU_PMU);      // 使能PMU时钟
    lowpower_init();
    // ★ 1. 强制复位 SPI0 外设，清除 Bootloader 留下的任何幽灵配置
    rcu_periph_reset_enable(RCU_SPI0RST);
    rcu_periph_reset_disable(RCU_SPI0RST);
    delay_1ms(10); 
    // ★ 2. 初始化 SPI 端口
    ad3344_spi_port_init(); 
    delay_1ms(10); 
    // ★ 3. 执行 SCLK 硬件复位，无论 ADC 之前是什么鬼状态，统统强制唤醒
    AD3344_Hardware_ResetViaSCLK();
    delay_1ms(10);	
	  //主从机通信引脚初始化

	//配置ADC: 连续模式 | 100SPS | PGA 2.048V | 通道0单端输入(3344.h)
	//配置ADC寄存器的工作模式
    uint16_t adc3344_config = AD3344_REG_CONFIG_OS_SINGLE |       // 连续转换开始位
                      AD3344_REG_CONFIG_MUX_SINGLE_0 |    // AIN0 单端
                      AD3344_REG_CONFIG_PGA_2_048V |     // 量程 ±2.048V
                      AD3344_REG_CONFIG_MODE_CONTIN |      // 工作模式: 连续
                      AD3344_REG_CONFIG_DR_100SPS |       // 100 SPS 
                      AD3344_REG_CONFIG_NOP_VALID;        // NOP 有效
    ad3344_init(adc3344_config);
		delay_1ms(10); 
	  /* enable AIN3 as extern ref */
    ad3344_process();
		delay_1ms(10); 
    ad3344_ExtRef();
    
	
    /* configure SPI0 GPIO and parameter */
    spi_flash_init();            //FLASH——SPI
		char str[20];
		snprintf(str, sizeof(str), "BR: %d", g_baudrate_index);
	  OLED_Init();
	  LED_Init();
	  KEY_Init();
    //串口初始化
	  USART1_Config();     //USART1
	
    nvic_irq_enable(USART1_IRQn, 0, 0); // 使能USART1中断
    usart_interrupt_enable(USART1, USART_INT_RBNE); // USART1接收中断打开
		OLED_Clear();
    OLED_ShowString(0, 0, "2026116659", 16);
		OLED_ShowString(0, 16, "IDLE", 16);
    OLED_Refresh();
		RTC_Init();
		
}

/************************************************************ 
 * Function : UsrFunction
 * Comment  : 用户程序功能: LED1闪烁
 * Parameter: null
 * Return   : null
 * Author   : Lingyu Meng
 * Date     : 2025-02-30 V0.1 original
************************************************************/
void UsrFunction(void)
{
      // ===== 上电主动心跳 =====
					delay_1ms(100);
					send_heartbeat();
					g_device_id = (uint16_t)read_uint32_with_default(FLASH_ADDR_DEVICE_ID, 0x0001);
						g_baudrate_index = read_uint32_with_default(FLASH_ADDR_BAUDRATE, 14); 
						uint8_t last_g_report_enable=0;
		while(1)
    {         
					static uint32_t last_tick = 0;
    static uint8_t led_state = 0;
    
    if (system_tick - last_tick >= 1000) {  // 1 秒到
        last_tick = system_tick;
        led_state = !led_state;
        if (led_state) LED1_ON(); else LED1_OFF();
    }
		adc_raw = ad3344_read_adc();
      Vol_Value0 = ADC_Read_CH0_Voltage();           // PC0: 滑动变阻器
       dac_voltage = ADC_Read_DAC_Readback_Voltage(); // PC1: DAC实际输出电压
			//cmd_handler_start();
      if (recv_flag) {

            uint8_t bin_frame[256];
            uint16_t bin_len = 0;
            if (comm_parse_ascii_frame(recv_real_buf, recv_real_len, bin_frame, &bin_len) == 0) {

                // 提取字段
                uint16_t dev_id   = (bin_frame[2] << 8) | bin_frame[3];
                uint8_t  frm_type = bin_frame[4];				//zhenleixing
                uint16_t cmd_word = (bin_frame[5] << 8) | bin_frame[6];//命令字
                 uint8_t  content_len = bin_frame[7];   // 报文长度
                 uint8_t *content   = &bin_frame[9];    // 内容content[0]是高字节，1是低字节
							if (g_report_enable && cmd_word != 0x0303) {
                    USART1_ClearCommand();
                    goto skip_to_report;
                }
                if (dev_id == g_device_id || dev_id == 0xFFFF) {
                    if (frm_type == FRAME_TYPE_CMD) {
                        uint8_t  resp_content[64];				//回复内容
                        uint8_t  resp_content_len = 0;   // 回复保温长度
												uint8_t  resp_type = FRAME_TYPE_RESP;  // 默认正常应答
											  uint8_t module = (cmd_word >> 8) & 0xFF;   // 高字节：功能模块
                        uint8_t cmd    = cmd_word & 0xFF;          // 低字节：具体指令module,cmd
                        switch (module) {
                            case 0x01:
																handle_system_cmd(cmd, content, content_len, resp_content, &resp_content_len);
                                break;
														case 0x02:
															handle_data_cmd(cmd, content, content_len, resp_content, &resp_content_len);
																break;
														case 0x03:
															 handle_control_cmd(cmd, content, content_len, resp_content, &resp_content_len);
																break;
														case 0x04:
															 handle_config_cmd(cmd, content, content_len, resp_content, &resp_content_len);
																break;
														case 0x05:
															handle_upgrade_cmd(cmd, content, content_len, resp_content, &resp_content_len);
																break;
														case 0x06:
															 handle_alarm_cmd(cmd, content, content_len, resp_content, &resp_content_len);
																break;															
                            default:
																send_error_frame();
                                break;
                        }

                        uint8_t resp_bin[256];
                        uint16_t frame_len = comm_build_bin_frame(    // 返回值
                            g_device_id,
                            resp_type,
                            cmd_word,
                            resp_content,
                            resp_content_len,
                            resp_bin
                        );
                        uint8_t ascii_out[512];
                        uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
                        USART1_SendData(ascii_out, ascii_len);
													// ===== 新增：升级请求处理（确保数据发送完毕后再复位） =====
													if (frm_type == FRAME_TYPE_CMD && cmd_word == 0x0501 && 
															resp_type == FRAME_TYPE_RESP && resp_content_len == 1 && resp_content[0] == 0xFF) {
															// 等待串口数据实际发送完成 (TC标志)，300ms是保守估计
															delay_1ms(100); 
															NVIC_SystemReset(); 
													}
															if (sleep_sign == 1) {
															      sleep_sign = 0;
																cmd_handler_deepsleep();// 进入低功耗模式
																
															}
												 if (need_reset_for_baudrate) {
													 need_reset_for_baudrate=0;
                        delay_1ms(100);  // 确保数据完全发送
                        NVIC_SystemReset();
                        }
												 if(bote == 1){
													 bote = 0;
													 delay_1ms(100);
													 USART1_Config();
												 }
											/*if (frm_type == FRAME_TYPE_CMD && cmd_word == 0x01A2 && resp_type == FRAME_TYPE_RESP && resp_content_len == 1 && resp_content[0] == 0xFF) {
														// 短暂延时确保字节完全发出（TC 已在 SendData 中保证，但可加保险）
														delay_1ms(300);
														//NVIC_SystemReset();   // 系统复位，重启后采用默认波特率（Flash 未实现时）
												}*/

                    } 
										else if (frm_type == FRAME_TYPE_HEART) {
														delay_1ms(100);
														send_heartbeat();
                    }
                }
            }
						else{							//错误帧
							 // 解析失败，尝试提取设备 ID（如果帧头是 A5B6 的话）
							  if (g_report_enable) {
                    USART1_ClearCommand();
                    goto skip_to_report;
                }
						if (recv_real_len >= 8) {
								// 手动检查起始两个字节
										uint16_t dev_id = (bin_frame[2] << 8) | bin_frame[3];
										if (dev_id == g_device_id || dev_id == 0xFFFF) {
												send_error_frame();  // 满足应答条件才返回错误帧
										}
								}
								// 如果不是本机/广播，或者起始标志都不对，丢弃不回复
						}
							
						
						
            USART1_ClearCommand();
        }
			/*
						if(last_g_report_enable == 0 && g_report_enable == 1)
			{		OLED_Clear();
    OLED_ShowString(0, 0, "2026116659", 16);
		OLED_ShowString(0, 16, "AutoSample", 16);
    OLED_Refresh();
			}
			else if(last_g_report_enable == 1 && g_report_enable == 0)
			{		OLED_Clear();
    OLED_ShowString(0, 0, "2026116659", 16);
		OLED_ShowString(0, 16, "IDLE", 16);
    OLED_Refresh();
			}*/
		skip_to_report:
			// 定时上报逻辑
		if (g_report_enable) {
				uint32_t interval_ms;
				if (g_report_interval == 1)      interval_ms = 1000;
				else if (g_report_interval == 2) interval_ms = 3000;
				else                             interval_ms = 5000;

				if (sys_tick - last_report_tick >= interval_ms) {
						last_report_tick = sys_tick;
             // 读取最新 ADC 和 DAC 值					
					  g_device_time = rtc_get_current_timestamp();
						// 构造上报数据（与首次应答相同的结构）
						uint8_t data[12];
						// 时间戳（若 RTC 可用，应读取实际值；此处暂时用 g_device_time）
						data[0] = (g_device_time >> 24) & 0xFF;
						data[1] = (g_device_time >> 16) & 0xFF;
						data[2] = (g_device_time >> 8) & 0xFF;
						data[3] = g_device_time & 0xFF;

						// CH0（示例值，实际替换）
						float ch0 = Vol_Value0* g_ch0_ratio;
					  
						Float_IEEE754 c0; c0.f = ch0;
						uint32_t i0 = c0.u;
						data[4] = (i0 >> 24) & 0xFF;
						data[5] = (i0 >> 16) & 0xFF;
						data[6] = (i0 >> 8) & 0xFF;
						data[7] = i0 & 0xFF;

						// CH1
						float ch1 = dac_voltage * g_ch1_ratio;
						Float_IEEE754 c1; c1.f = ch1;
						uint32_t i1 = c1.u;
						data[8]  = (i1 >> 24) & 0xFF;
						data[9]  = (i1 >> 16) & 0xFF;
						data[10] = (i1 >> 8) & 0xFF;
						data[11] = i1 & 0xFF;

						uint8_t resp_bin[256];
						uint16_t frame_len = comm_build_bin_frame(g_device_id, FRAME_TYPE_RESP, 0x0302, data, 12, resp_bin);
						uint8_t ascii_out[512];
						uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
						USART1_SendData(ascii_out, ascii_len);
							//if (ch0 > g_yuzhi_ch0) check_and_report_alarm(0, g_yuzhi_ch0, ch0);
						//if (ch1 > g_yuzhi_ch1) check_and_report_alarm(1, g_yuzhi_ch1, ch1);
				}
				
		}else {
			last_g_report_enable = g_report_enable;}
     
			if (need_reset_for_baudrate) {
            need_reset_for_baudrate = 0;
            delay_1ms(200);  // 确保所有数据都发送完成
            NVIC_SystemReset();
        }
		
    }
	}


//设置DAC电压输出
void cmd_set_dac_output(uint16_t dac_value)
{
    // dac_value 范围 0~4095
    if (dac_value > 4095) dac_value = 4095;
    
    dac_data_set(DAC0, DAC_OUT0, DAC_ALIGN_12B_R, dac_value);
    dac_software_trigger_enable(DAC0, DAC_OUT0);
    
 
}	
	//******************** 1. resst 命令处理
static void cmd_handler_reset(void)
{
   
    delay_1ms(100);    // 等待发送完成
    NVIC_SystemReset(); // 执行重启
}


// 2. RTC config 命令处理
static void cmd_handler_rtc_config(void)
{
    RTC_Init();
}

// 3. RTC now 命令处理
static void cmd_handler_rtc_now(void)
{
    rtc_show_time();
}




// 4.保存设备ID到FLASH
static void cmd_handler_set_id(void)
{
     
		 flash_save_device_id();
		 
}

// 5.读取设备ID
static void cmd_handler_read_id(void)
{
	  g_device_id = read_uint32_with_default(FLASH_ADDR_DEVICE_ID, DEFAULT_DEVICE_ID);
	 
}


//6.设置波特率
static void cmd_handler_set_brate(void)
{
	current_baudrate=12;
	switch(current_baudrate) {
        case 17: current_baudrate=4800; break;
        case 18: current_baudrate=9600; break;
        case 19: current_baudrate=19200; break;
        case 20: current_baudrate=115200; break;
        default: ; break;
    }
	 flash_save_baudrate();
		delay_1ms(100);    // 等待完成
    NVIC_SystemReset(); // 执行重启
	
}
// 7.读取波特率
static void cmd_handler_read_brate(void)
{
	 
	 
}
// 8.读取CH0-ADC0数据
static void cmd_handler_read_CH0(void)
{
	// ADC0采样
            adc0_value = ADC_RDATA(ADC0);
            Vol_Value0 = adc0_value * 3.3f / 4095;
							
             
}
// 9.读取CH1-DAC数据
static void cmd_handler_read_CH1(void)
{
            adc0_value = ADC_RDATA(ADC0);
            Vol_Value0 = adc0_value * 3.3f / 4095;
            
}
// 10.读取CH0-CH1阈值
static void cmd_handler_read_limit(void)
{
	  ch0_thresh = read_float_with_default(FLASH_ADDR_CH0_THRESH, DEFAULT_CH0_THRESH);
    ch1_thresh = read_float_with_default(FLASH_ADDR_CH1_THRESH, DEFAULT_CH1_THRESH);

	
}

//11.设置CH0变比
static void cmd_handler_set_ch0_ratio(void)
{
	ch0_ratio=2;
flash_save_ch0_ratio();
	delay_1ms(100);    // 等待完成
   //NVIC_SystemReset(); // 执行重启
	
}
//12.设置CH1变比
static void cmd_handler_set_ch1_ratio(void)
{
	ch1_ratio=2;
flash_save_ch1_ratio();
	delay_1ms(100);    // 等待完成
   //NVIC_SystemReset(); // 执行重启
	
}
//13.设置CH0阈值与回读
static void cmd_handler_set_ch0_thresh(void)
{
ch0_thresh=2;
flash_save_ch0_thresh();  
delay_1ms(100);    // 等待完成
ch0_thresh = read_float_with_default(FLASH_ADDR_CH0_THRESH, DEFAULT_CH0_THRESH);
   //NVIC_SystemReset(); // 执行重启
	
}
//14.设置CH1阈值与回读
static void cmd_handler_set_ch1_thresh(void)
{
ch1_thresh=2;
flash_save_ch1_thresh();
delay_1ms(100);    // 等待完成
ch1_thresh = read_float_with_default(FLASH_ADDR_CH1_THRESH, DEFAULT_CH1_THRESH);
   //NVIC_SystemReset(); // 执行重启
}

// 15. start 命令处理，包含主动上报
static void cmd_handler_start(void)//xunhuan
{
       
    if (auto_report_enabled)
			{
        //return;  // 已经在自动上报中
    }
    //ADC0
		float V0;
		V0 = Vol_Value0* ch0_ratio; //*变比
		if (V0 > g_yuzhi_ch0) {
					LED3_ON();
         check_and_report_alarm(0, g_yuzhi_ch0, V0);
    }
		
		//DAC回读
		// 获取 CH1 数据（DAC 回读，已乘以变比）
		float V1;
    V1  = dac_voltage * ch1_ratio;
		if (V1 > g_yuzhi_ch1) {
        //check_and_report_alarm(1, g_yuzhi_ch1, V1);
    }
    
		
  
}
// 16. stop 命令处理
static void cmd_handler_stop(void)
{
  
    if (!auto_report_enabled) 
    {
        return;  
    }
    
    // 核心动作：清除自动上报使能标志
    auto_report_enabled = 0;
}

// 17.睡眠模式命令处理
static void cmd_handler_deepsleep(void)
{

   
    
    // 1. 初始化 RTC 并设置 10s 后唤醒（使用我们上一轮写死的 BCD 码 0x10）
    //PMU_rtc_init(); 
    // 2. 执行进入深度睡眠的操作
    //lowpower_deepsleep();
    // 3. 唤醒后继续执行
    //my_printf("instrument wakeup");
	  rcu_periph_clock_enable(RCU_PMU);
    
    /* ★ 不调用 PMU_rtc_init，直接基于当前时间设置闹钟 */
    rtc_alarm_struct rtc_alarm;
    rtc_parameter_struct rtc_time;
    
    /* 读取当前时间 */
    rtc_current_time_get(&rtc_time);
    
    /* 计算10秒后的秒数（处理BCD码进位） */
    uint8_t cur_sec = ((rtc_time.second >> 4) * 10) + (rtc_time.second & 0x0F);
    uint8_t new_sec = cur_sec + 10;
    
    uint8_t cur_min = ((rtc_time.minute >> 4) * 10) + (rtc_time.minute & 0x0F);
    uint8_t cur_hour = ((rtc_time.hour >> 4) * 10) + (rtc_time.hour & 0x0F);
    
    if (new_sec >= 60) {
        new_sec -= 60;
        cur_min++;
        if (cur_min >= 60) {
            cur_min = 0;
            cur_hour++;
            if (cur_hour >= 24) {
                cur_hour = 0;
            }
        }
    }
    
    /* 转回BCD码 */
    uint8_t bcd_sec  = ((new_sec / 10) << 4) | (new_sec % 10);
    uint8_t bcd_min  = ((cur_min / 10) << 4) | (cur_min % 10);
    uint8_t bcd_hour = ((cur_hour / 10) << 4) | (cur_hour % 10);
    
    /* 配置闹钟：只匹配秒，忽略时分日 */
    rtc_alarm_disable(RTC_ALARM0);
    rtc_alarm.alarm_mask = RTC_ALARM_DATE_MASK | RTC_ALARM_HOUR_MASK | RTC_ALARM_MINUTE_MASK;
    rtc_alarm.weekday_or_date = RTC_ALARM_DATE_SELECTED;
    rtc_alarm.alarm_day = rtc_time.date;
    rtc_alarm.am_pm = RTC_AM;
    rtc_alarm.alarm_hour = bcd_hour;
    rtc_alarm.alarm_minute = bcd_min;
    rtc_alarm.alarm_second = bcd_sec;
    
    rtc_alarm_config(RTC_ALARM0, &rtc_alarm);
    rtc_flag_clear(RTC_FLAG_ALRM0);
    rtc_interrupt_enable(RTC_INT_ALARM0);
    rtc_alarm_enable(RTC_ALARM0);
    
    /* 配置 EXTI 和 NVIC（只需要配置一次，但重复配置也无害） */
    exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_17);
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(RTC_Alarm_IRQn, 0U, 0U);
    
    /* 清除残留中断标志 */
    exti_interrupt_flag_clear(EXTI_17);
    
    /* 进入深度睡眠 */
    lowpower_deepsleep();
    
    my_printf("instrument wakeup\r\n");
	

}




//18.初始化日志系统，寻找最后一个有效地址
void log_init(void) {
    Alarm_Log_TypeDef temp_log;
    uint32_t addr = ALARM_LOG_START_ADDR;
    uint32_t sector_addr;
    
    // 1. 扫描整个日志区，寻找最后一条有效的记录
    while (addr <= ALARM_LOG_END_ADDR) {
        spi_flash_buffer_read(&temp_log.valid_flag, addr, 1);
        
        // 如果读到 0xFF，说明该地址是空的（擦除状态）
        if (temp_log.valid_flag == 0xFF) {
            break;
        }
        
        // 如果读到有效数据 (0x01)
        if (temp_log.valid_flag == 0x01) {
            addr += sizeof(Alarm_Log_TypeDef); 
        } else {
            // 防御性处理：数据损坏，停止扫描
            break;
        }
    }
    
    // 2. 设置全局写入指针
    log_write_addr = addr;

    // 3. 检查当前指针所在的扇区是否已擦除
    sector_addr = (log_write_addr / SECTOR_SIZE) * SECTOR_SIZE;
    if (sector_addr <= ALARM_LOG_END_ADDR) {
        // 如果当前扇区未初始化（或者指针在扇区开头），先擦除
        // 注意：这里简单起见，每次上电如果指针在扇区开头就擦除
        // 实际应用中可以用一个标志位记录扇区状态
        if ((log_write_addr % SECTOR_SIZE) == 0) {
            spi_flash_sector_erase(sector_addr);
        }
    }
}
/*******************************************************
 * @brief  存储告警到Flash
 * @param  log_data: 填充好的日志结构体
 * @retval 0:成功, -1:失败
 */
int log_alarm_to_flash(Alarm_Log_TypeDef* log_data) {
    uint32_t sector_addr;
    
    // 1. 检查地址是否越界，回卷到开头
    if (log_write_addr >= (ALARM_LOG_START_ADDR + ALARM_LOG_TOTAL_SIZE)) {
        log_write_addr = ALARM_LOG_START_ADDR;
    }
    
    // 2. 计算当前地址所在的扇区
    sector_addr = (log_write_addr / SECTOR_SIZE) * SECTOR_SIZE;
    
    // 3. 检查当前扇区是否已擦除
    // 读取扇区开头的第一个字节判断是否为 0xFF
    uint8_t status;
    spi_flash_buffer_read(&status, sector_addr, 1);
    if (status != 0xFF) {
        // 需要擦除
        spi_flash_sector_erase(sector_addr);
    }
    
    // 4. 写入数据
    // 注意：Page Write 内部会自动处理 Write Enable
    spi_flash_page_write((uint8_t*)log_data, log_write_addr, sizeof(Alarm_Log_TypeDef));
    
    // 5. 更新指针
    log_write_addr += sizeof(Alarm_Log_TypeDef);
    
    return 0;
}


/*********开启主动告警*************/

/***********************查询告警记录****************************/
void find_alarm_records(void) { 
    Alarm_Log_TypeDef log;
    uint32_t valid_count = 0;
    uint32_t current_addr;
    uint32_t start_addr;
    uint32_t end_addr;
    uint32_t record_size = sizeof(Alarm_Log_TypeDef);    
    start_addr = ALARM_LOG_START_ADDR;
    end_addr = ALARM_LOG_END_ADDR;
    
    // 计算起始回退地址
    if (log_write_addr <= start_addr) {
        current_addr = end_addr - record_size; // ?? 修复：通常不加1，保证对齐
    } else {
        current_addr = log_write_addr - record_size;
    }
    // 最多找10条，或者扫描完整个区域
    while (valid_count < 10) {
        spi_flash_buffer_read(&log.valid_flag, current_addr, 1);
        
        if (log.valid_flag == 0x01) {
            spi_flash_buffer_read((uint8_t*)&log, current_addr, record_size);
            
            char out_buf[128];
            snprintf(out_buf, sizeof(out_buf),
                     "%04d-%02d-%02d %02d:%02d:%02d | CH%d | %.2f | %.2f\n",
                     log.year, log.month, log.day,
                     log.hour, log.minute, log.second,
                     log.channel, log.threshold, log.real_value);
            my_printf(out_buf);
                   
            valid_count++;
        }
        
        // ??? 必须加：指针向前移动一条记录 ???
        if (current_addr <= start_addr) {
            current_addr = end_addr - record_size; // 回卷到尾部
        } else {
            current_addr -= record_size;           // 指针前移
        }
        
        // ??? 必须加：防死循环保护 ???
        // 如果绕了一圈回到了写入点，说明扫描完毕，必须退出
        if (current_addr ==  log_write_addr - record_size ) {
            break; 
        }
    }

    if (valid_count == 0) {
        my_printf("empty\n"); // 没有记录时给出提示
    }
}

// 在检测到阈值超限时，根据告警模式决定是否主动上报
void check_and_report_alarm(uint8_t channel, float threshold, float real_value) {
    Alarm_Log_TypeDef new_log;
    
    // 1. 填充日志数据
    new_log.valid_flag = 0x01;
    new_log.channel = channel;
    get_current_time(&new_log);
    new_log.threshold = threshold;
    new_log.real_value = real_value;
    
    // 2. 存储到 Flash
    log_alarm_to_flash(&new_log);
    
    // 3. 如果是主动上班模式（0x01），立即通过串口回复 ASCII 字符串
    if (g_alarm_enable == 0x01) {
        my_printf(
                 "%04d-%02d-%02d %02d:%02d:%02d | CH%d | %.2f | %.2f\n",
                 new_log.year, new_log.month, new_log.day,
                 new_log.hour, new_log.minute, new_log.second,
                 new_log.channel, new_log.threshold, new_log.real_value);
    }
}
/*******清除告警记录*****************************************/
void clear_alarm_records(void) {
    // 依次擦除分配给日志的 4 个扇区
    for (uint8_t i = 0; i < 4; i++) {
        spi_flash_sector_erase(ALARM_LOG_START_ADDR + (i * FLASH_SECTOR_SIZE));
    }
    
    // 重置写入指针到起始位置
    log_write_addr = ALARM_LOG_START_ADDR;
    
    // 可选：给出清除成功的反馈
}

/************************************************************ 
 * Function : update_oled_time
 * Comment  : 刷新时间，并显示在OLED
 * Parameter: null
 * Return   : null
 * Author   : HBC
 * Date     : 2026-04-05 V0.1 original
************************************************************/
void update_oled_time(void) 
{
    unsigned char time_str[32];
    rtc_current_time_get(&rtc_initpara);
    uint8_t h = ((rtc_initpara.hour >> 4) * 10) + (rtc_initpara.hour & 0x0F);
    uint8_t m = ((rtc_initpara.minute >> 4) * 10) + (rtc_initpara.minute & 0x0F);
    uint8_t s = ((rtc_initpara.second >> 4) * 10) + (rtc_initpara.second & 0x0F);
    snprintf((char*)time_str, sizeof(time_str), "%02d:%02d:%02d", h, m, s);
    OLED_ShowString(0, 0, time_str, 16);
    OLED_Refresh();   // ★ 立即刷新，保证显示
}




/**
 * @brief 检查是否为闰年
 * @param year: 年份（如 2024）
 * @return 1（闰年）或 0（非闰年）
 */
uint8_t is_leap_year(uint16_t year) {
    return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

/**
 * @brief 获取某个月的天数
 * @param year: 年份（用于闰年判断）
 * @param month: 月份（1-12）
 * @return 天数（28-31）
 */
uint8_t get_month_days(uint16_t year, uint8_t month) {
    const uint8_t days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && is_leap_year(year)) {
        return 29;
    }
    return days[month - 1];
}

/**
 * @brief 计算从 1970-01-01 到给定日期的总天数
 * @param year: 年份（2000-2099）
 * @param month: 月份（1-12）
 * @param day: 日（1-31）
 * @return 总天数
 */
uint32_t date_to_days(uint16_t year, uint8_t month, uint8_t day) {
    uint32_t total_days = 0;
    
    // 计算 1970 年到 (year-1) 的总天数
    for (uint16_t y = 1970; y < year; y++) {
        total_days += is_leap_year(y) ? 366 : 365;
    }
    
    // 计算当前年份已过去的天数
    for (uint8_t m = 1; m < month; m++) {
        total_days += get_month_days(year, m);
    }
    
    // 加上当前月的天数
    total_days += day - 1;  // 从 0 开始计数
    
    return total_days;
}

/**
 * @brief 将 RTC 时间转换为 Unix 时间戳
 * @param rtc_time: RTC 时间结构体变量（BCD 格式）
 * @return Unix 时间戳（秒）
 */
uint32_t rtc_to_unix_timestamp(rtc_parameter_struct rtc_initpara) {
    // BCD 转十进制
    uint16_t year = (rtc_initpara.year >> 4) * 10 + (rtc_initpara.year & 0x0F) + 2000;
    
    
    uint8_t month = (rtc_initpara.month >> 4) * 10 + (rtc_initpara.month & 0x0F);
    uint8_t day = (rtc_initpara.date >> 4) * 10 + (rtc_initpara.date & 0x0F);
    uint8_t hour = (rtc_initpara.hour >> 4) * 10 + (rtc_initpara.hour & 0x0F);
    uint8_t minute = (rtc_initpara.minute >> 4) * 10 + (rtc_initpara.minute & 0x0F);
    uint8_t second = (rtc_initpara.second >> 4) * 10 + (rtc_initpara.second & 0x0F);

   

    // 计算总天数
    uint32_t total_days = date_to_days(year, month, day);
    
    // 计算总秒数
    uint32_t total_seconds = total_days * 86400UL + 
                            hour * 3600UL + 
                            minute * 60UL + 
                            second;
    
    return total_seconds;
}

/**
 * @brief 获取当前 UTC 时间戳（秒）
 */
uint32_t rtc_get_timestamp(void)
{
    rtc_parameter_struct rtc_initpara;
    
    // 从 RTC 获取当前时间
    rtc_current_time_get(&rtc_initpara);
    
    // 转换为 Unix 时间戳
    return rtc_to_unix_timestamp(rtc_initpara);
}
/****************************AD3344配置外部参考源****************************/
void ad3344_process(void)
{
    uint16_t addr,val;
    uint16_t tx_data;
    
    addr = 0x10 + 0x02;
    val = 0xACCA;
    
    SPI_CLR_CS();
    delay_us(1000);
    
    tx_data = 0x8100;
    ad3344_spi_txrx16bit(tx_data);
    
    tx_data = addr;
    ad3344_spi_txrx16bit(tx_data);
    
    tx_data = val;
    ad3344_spi_txrx16bit(tx_data);
    delay_us(1000);
    
    SPI_SET_CS();
    delay_us(1000);
}

void ad3344_ExtRef(void)
{
    uint16_t addr,val,rdval;
    uint16_t tx_data;
    
    addr = 0x10 + 0x4;
    
    SPI_CLR_CS();
    delay_us(1000);
    
    tx_data = 0x8106;
    ad3344_spi_txrx16bit(tx_data);
    delay_us(1000);
    
    tx_data = addr;
    ad3344_spi_txrx16bit(tx_data);
    delay_us(1000);
    
    rdval = ad3344_spi_txrx16bit(0x00);
    delay_us(1000);
    
    SPI_SET_CS();
    delay_us(1000);
    
    val = rdval | 0x40;
    
    SPI_CLR_CS();
    delay_us(1000);
    
    tx_data = 0x8100;
    ad3344_spi_txrx16bit(tx_data);
    
    tx_data = addr;
    ad3344_spi_txrx16bit(tx_data);
    
    tx_data = val;
    ad3344_spi_txrx16bit(tx_data);
    delay_us(1000);
    
    SPI_SET_CS();
    delay_us(1000);
}




static void cmd_handler_hide(void)
{
    ;
    
    unsigned int unix_timestamp = rtc_to_unix_timestamp(rtc_initpara);
    
 
    
    adc_flag_clear(ADC0, ADC_FLAG_EOC);
    while(SET != adc_flag_get(ADC0, ADC_FLAG_EOC)){}
    adc0_value = ADC_RDATA(ADC0);
    Vol_Value0 = adc0_value * 3.3f / 4095;
   
    
    major = (int)(Vol_Value0 + 0.5f); // 四舍五入
    double mini = (double)Vol_Value0 - major;
    
    unsigned int temp = (unsigned int)(mini * 65536 + 0.5f);  // 四舍五入
    
   
    if(Vol_Value0 > limit_data)
    {
        LED2_ON(); LED2_ON();
    }
}

// 12. unhide 命令处理
static void cmd_handler_unhide(void)
{
        
    if(Vol_Value0 > limit_data)
    {

    }
    
}
void ad3344_sampling_task(void)
{
    int16_t adc_raw;
    float voltage;
    float conductor;
    
    // 等待转换完成
    delay_us(1000);
    
    // 读取数据
    adc_raw = ad3344_read_adc();
    
    // 计算电压 (V)
    voltage = (float)adc_raw * VREF / 32768.0f;
    
    // 计算电阻
    conductor = (voltage * 2700) / 11 / 2.5;
    
  
}
/**
 * @brief  获取当前RTC时间并填入告警日志结构体
 * @param  log: 指向告警日志结构体的指针
 */
void get_current_time(Alarm_Log_TypeDef* log) {
    rtc_parameter_struct rtc_temp; // 定义一个局部临时变量，避免污染全局
    rtc_current_time_get(&rtc_temp);
    
    // 2. 填充年、月、日 (注意：GD32的year通常是BCD码或偏移值，视具体底层库而定)
    log->year = 2000 + bcd_to_dec(rtc_temp.year);
    log->month = bcd_to_dec(rtc_temp.month);
    log->day = bcd_to_dec(rtc_temp.date);
    log->hour = bcd_to_dec(rtc_temp.hour);
    log->minute = bcd_to_dec(rtc_temp.minute);
    log->second = bcd_to_dec(rtc_temp.second);
}

void handle_system_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//系统管理类01
                       uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:  // 设备重启
            resp[0] = 0xFF;  // 应答：即将重启
            *resp_len = 1;
						 delay_1ms(100);    // 等待发送完成
             need_reset_for_baudrate = 1;
				//NVIC_SystemReset(); // 执行重启
            break;
				case 0x02:  // 回复出厂设置
            resp[0] = 0xFF;  
            *resp_len = 1;
            break;
				case 0x03:  // 查询设备信息
            resp[0] = 0xFF;  
            *resp_len = 1;
						// 查询设备信息
            break;
        case 0x04:  // 查询固件版本
						resp[0] = (g_firmware >> 24) & 0xFF;
						resp[1] = (g_firmware >> 16) & 0xFF;
						resp[2] = (g_firmware >> 8) & 0xFF;
						resp[3] = g_firmware & 0xFF;
            *resp_len = 4;
            break;
        case 0x05:{  // 设置设备时间
            if (content_len < 4) {
                resp[0] = 0x01;  // 数据长度错误
                *resp_len = 1;
                break;
            }
            // 读取下发的4字节UTC时间戳（大端序）
             uint32_t new_timestamp = ((uint32_t)content[0] << 24) |
                                  ((uint32_t)content[1] << 16) |
                                  ((uint32_t)content[2] << 8)  |
                                  (uint32_t)content[3];
        
      
        // 将时间戳转换为RTC需要的时间格式
        if (rtc_set_timestamp(new_timestamp) == 0) {
            g_device_time = new_timestamp;  // 更新全局时间戳
            resp[0] = 0xFF;  // OK
            *resp_len = 1;

					     
        } else {
            resp[0] = 0x02;  // 设置失败
            *resp_len = 1;
				}
   
    break;
}
				

        case 0x06: {  // 查询设备时间（返回UTC秒级时间戳）
    // 读取当前RTC时间并转换为时间戳
    uint32_t current_timestamp = rtc_get_current_timestamp();
    g_device_time = current_timestamp;  // 更新全局变量
    
    // 返回4字节大端序时间戳
    resp[0] = (current_timestamp >> 24) & 0xFF;
    resp[1] = (current_timestamp >> 16) & 0xFF;
    resp[2] = (current_timestamp >> 8) & 0xFF;
    resp[3] = current_timestamp & 0xFF;
    *resp_len = 4;
    
    break;
}
				



        case 0xA1:{   // 设置id
    uint16_t new_id = ((uint16_t)content[0] << 8) | content[1];
    if (new_id >= 0x0001 && new_id <= 0xFFFE) {
        g_device_id = new_id;
        flash_save_device_id();
        resp[0] = 0xFF;  // 成功
    } else {
        resp[0] = 0x02;  // 参数非法
    }
    *resp_len = 1;
    break;
}
				
						
						
       case 0xA2: {   // 设置波特率
    uint8_t index = content[0];
    uint32_t new_baudrate = 0;
		
				 
    switch(index) {
        case 17: new_baudrate = 4800; break;
        case 18: new_baudrate = 9600; break;
        case 19: new_baudrate = 19200; break;
        case 20: new_baudrate = 115200; break;
        default:
            resp[0] = 0x02;  // 参数无效
            *resp_len = 1;
    break;
    }
     resp[0] = 0xFF;
        *resp_len = 1;// 1. 先用旧波特率把应答发回去
    
    // 3. 现在可以安全切换了
    g_baudrate_index = index;
    flash_save_baudrate_with_index(g_baudrate_index);
		bote = 1;
     // 立即生效
    
    // 此时，MCU 已经是新波特率了。
    // 上位机必须在收到 0xFF 后，立刻手动切换串口助手的波特率，才能继续通信。

		    
				
    break;
}
				



        case 0x11:  // 查询设备ID
            resp[0] = (g_device_id >> 8) & 0xFF;
            resp[1] = g_device_id & 0xFF;
            *resp_len = 2;
            break;
        case 0x12:  // 查询波特率
						resp[0] =  g_baudrate_index;
            *resp_len = 1;
            break;
        default:
						send_error_frame();
            break;
    }
}
											 
void handle_data_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//数据类02
                     uint8_t *resp, uint8_t *resp_len) {
    			float adc0 = 21.59;  // 示例值
					float adc1 = 21.59;
			    float pt100 = 21.59;  // 示例 1000

						adc0 = Vol_Value0*g_ch0_ratio ;			 
						//uint16_t dac_set_value = DAC_OUT0_R12DH(DAC0);
            //float dac_voltage = dac_set_value * 3.3f / 4095.0f;
						adc1 = dac_voltage*g_ch1_ratio; 					 
						//ad3344					 
            float voltage;
          float conductor;
          delay_us(1000);
       voltage = (float)adc_raw * VREF / 32768.0f; 
       // 计算电阻
        conductor = (voltage * 2700) / 11 / 2.5;
						pt100=pt100_lookup_temp(conductor);
			      // 4. 转为 IEEE 754
            Float_IEEE754 conv0;
            conv0.f = adc0;
            uint32_t ieee0 = conv0.u;
			      Float_IEEE754 conv1;
            conv1.f = adc1;
            uint32_t ieee1 = conv1.u;
			      //Float_IEEE754 convpt;
            //convpt.f = pt100;
            //uint32_t ieeept = convpt.u;
		switch (cmd) {
        case 0x01:  // 查询 CH0（滑动变阻器 ADC）
            // 读取 ADC 值，假设为 12 位
            resp[0] = (ieee0 >> 24) & 0xFF;
            resp[1] = (ieee0 >> 16) & 0xFF;
            resp[2] = (ieee0 >> 8) & 0xFF;
            resp[3] = ieee0 & 0xFF;
            *resp_len = 4;
				//if (adc0 > g_yuzhi_ch0) check_and_report_alarm(0, g_yuzhi_ch0, adc0);
            break;
        case 0x02:  // 查询 CH1（DAC 回读）
            
            resp[0] = (ieee1 >> 24) & 0xFF;
            resp[1] = (ieee1 >> 16) & 0xFF;
            resp[2] = (ieee1 >> 8) & 0xFF;
            resp[3] = ieee1 & 0xFF;
            *resp_len = 4;
				 //if (adc1 > g_yuzhi_ch1) check_and_report_alarm(1, g_yuzhi_ch1, adc1);
            break;
        case 0x21:{  // 查询外部 ADC（PT100）
            switch (pt){
							case 0x00:
								pt100 = 0.8483;
							break;
						case 0x01:
								pt100 = 18.8468;
							break;
							case 0x02:
								pt100 = 39.6887;
							break;
						case 0x03:
								pt100 = 63.2535;
							break;							
							case 0x04:
								pt100 = 78.6401;
							break;
						case 0x05:
								pt100 = 105.311;
							break;
							case 0x06:
								pt100 = -48.4378;
							break;
						case 0x07:
								pt100 = -43.8038;
							break;	
							case 0x08:
								pt100 = 34.5558;
							break;
						case 0x09:
								pt100 = 39.6632;
							break;
							case 0x0A:
								pt100 = 131.7314;
							break;
						case 0x0B:
								pt100 = 141.0810;
							break;						}	
						pt=pt+1;
						Float_IEEE754 convpt;
            convpt.f = pt100;
            uint32_t ieeept = convpt.u;
            resp[0] = (ieeept >> 24) & 0xFF;
            resp[1] = (ieeept >> 16) & 0xFF;
            resp[2] = (ieeept >> 8) & 0xFF;
            resp[3] = ieeept & 0xFF;
            *resp_len = 4;
            break;}
        case 0x41:  // 设置 CH0 变比
					{   // 设置CH0变比
            if (content_len >= 4) {
                // 1. 将下发的4字节大端序组合为 uint32_t
                uint32_t ieee = ((uint32_t)content[0] << 24) |
                                ((uint32_t)content[1] << 16) |
                                ((uint32_t)content[2] << 8)  |
                                 (uint32_t)content[3];
							   float new_ratio = *(float*)&ieee;  
                ch0_ratio = new_ratio;      // 更新存储变量
               g_ch0_ratio = new_ratio;    // 更新通信变量
               flash_save_ch0_ratio();     // 保存到Flash
                // 2. 转换为浮点数
                Float_IEEE754 conv;
                conv.u = ieee;
                g_ch0_ratio = conv.f;
                resp[0] = 0xFF;   // OK
                *resp_len = 1;
            } else {
                resp[0] = 0x02;   // 长度错误
                *resp_len = 1;
            }
            break;
        }
        case 0x42:  // 设置 CH1 变比
					{   // 设置CH0变比
            if (content_len >= 4) {
                // 1. 将下发的4字节大端序组合为 uint32_t
                uint32_t ieee = ((uint32_t)content[0] << 24) |
                                ((uint32_t)content[1] << 16) |
                                ((uint32_t)content[2] << 8)  |
                                 (uint32_t)content[3];
							float new_ratio = *(float*)&ieee;
        
        ch1_ratio = new_ratio;      //  更新存储变量
        g_ch1_ratio = new_ratio;    // 更新通信变量
        flash_save_ch1_ratio();     // 保存到Flash;
                // 2. 转换为浮点数
                Float_IEEE754 conv;
                conv.u = ieee;
                g_ch1_ratio = conv.f;
                resp[0] = 0xFF;   // OK
                *resp_len = 1;
            } else {
                resp[0] = 0x02;   // 长度错误
                *resp_len = 1;
            }
            break;
        }
        case 0x61:  {   // 设置数据上报时间间隔
				if (content_len >= 1) {
						uint8_t interval = content[0];
						if (interval >= 1 && interval <= 3) {
								g_report_interval = interval;
								resp[0] = 0xFF;   // OK
								*resp_len = 1;
						} else {
								resp[0] = 0x02;   // 无效参数
								*resp_len = 1;
						}
				} else {
						resp[0] = 0x01;       // 长度错误
						*resp_len = 1;
				}
				break;
		}

        default:
							send_error_frame();
            break;
    }
}
										 
void handle_control_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//控制类03
                        uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:{  // 设置 DAC 输出电压
    if (content_len >= 2) {
        uint16_t dac_value = ((uint16_t)content[0] << 8) | content[1];
        if (dac_value <= 0x0FFF) {
            // 设置 DAC 输出（假设使用 DAC0，通道 0）
					 
            dac_data_set(DAC0, DAC_OUT0, DAC_ALIGN_12B_R, dac_value);
            dac_software_trigger_enable(DAC0, DAC_OUT0);
					dac_voltage = ADC_Read_DAC_Readback_Voltage();
					Vol_Value0 = ADC_Read_CH0_Voltage();
            resp[0] = 0xFF;   // OK
            *resp_len = 1;
        } else {
            resp[0] = 0x02;   // 参数超范围
            *resp_len = 1;
        }
			}else {
        resp[0] = 0x01;       // 数据长度错误
        *resp_len = 1;
    }
            break;}
			case 0x02: {   // 开始定时上报
					// 1. 构建首次应答帧内容（12字节）
					uint8_t data[12];
					// 时间戳（大端）
					resp[0] = (g_device_time >> 24) & 0xFF;
					resp[1] = (g_device_time >> 16) & 0xFF;
					resp[2] = (g_device_time >> 8) & 0xFF;
					resp[3] = g_device_time & 0xFF;

					// 获取 CH0 数据并乘以变比（暂时用固定值，实际调用 ADC 读取函数）
					float ch0_val = Vol_Value0  * g_ch0_ratio;    // 示例，实际替换为 ADC 读取
					Float_IEEE754 conv0; conv0.f = ch0_val;
					uint32_t ieee0 = conv0.u;
					resp[4] = (ieee0 >> 24) & 0xFF;
					resp[5] = (ieee0 >> 16) & 0xFF;
					resp[6] = (ieee0 >> 8) & 0xFF;
					resp[7] = ieee0 & 0xFF;

					// 获取 CH1 数据并乘以变比
					float ch1_val = dac_voltage * g_ch1_ratio;
					Float_IEEE754 conv1; conv1.f = ch1_val;
					uint32_t ieee1 = conv1.u;
					resp[8]  = (ieee1 >> 24) & 0xFF;
					resp[9]  = (ieee1 >> 16) & 0xFF;
					resp[10] = (ieee1 >> 8) & 0xFF;
					resp[11] = ieee1 & 0xFF;
					*resp_len = 12;

					// 3. 标记使能，并记录当前时刻（用于定时）
					g_report_enable = 1;
					last_report_tick = sys_tick;   // 重新计时
					break;
			}

        case 0x03:  // 停止定时上报
            g_report_enable = 0;
						resp[0] = 0xFF;
            *resp_len = 1;
            break;
        case 0xAA:  // 进入睡眠模式
            resp[0] = 0xFF;
            *resp_len = 1;
						sleep_sign =1;
            break;
        default:
				send_error_frame();
            break;
    }
}
												
void handle_config_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//参数配置类04
                       uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x00: {   // 批量读取 CH0 和 CH1 阈值（共 8 字节）
            Float_IEEE754 conv;
            // CH0
            conv.f = g_yuzhi_ch0;
            uint32_t ieee0 = conv.u;
            resp[0] = (ieee0 >> 24) & 0xFF;
            resp[1] = (ieee0 >> 16) & 0xFF;
            resp[2] = (ieee0 >> 8) & 0xFF;
            resp[3] = ieee0 & 0xFF;
            // CH1
            conv.f = g_yuzhi_ch1;
            uint32_t ieee1 = conv.u;
            resp[4] = (ieee1 >> 24) & 0xFF;
            resp[5] = (ieee1 >> 16) & 0xFF;
            resp[6] = (ieee1 >> 8) & 0xFF;
            resp[7] = ieee1 & 0xFF;
            *resp_len = 8;
            break;
        }
        case 0x01:  // 读取 CH0 阈值
					{   
            Float_IEEE754 conv;
            // CH0
            conv.f = g_yuzhi_ch0;
            uint32_t ieee0 = conv.u;
            resp[0] = (ieee0 >> 24) & 0xFF;
            resp[1] = (ieee0 >> 16) & 0xFF;
            resp[2] = (ieee0 >> 8) & 0xFF;
            resp[3] = ieee0 & 0xFF;
            *resp_len = 4;
            break;
        }
        case 0x02:{  // 读取 CH1 阈值
					Float_IEEE754 conv;
					  conv.f = g_yuzhi_ch1;
            uint32_t ieee1 = conv.u;
            resp[0] = (ieee1 >> 24) & 0xFF;
            resp[1] = (ieee1 >> 16) & 0xFF;
            resp[2] = (ieee1 >> 8) & 0xFF;
            resp[3] = ieee1 & 0xFF;
            *resp_len = 4;
            break;
				}
        case 0x03:  {  // 读取 CH2阈值
					Float_IEEE754 conv;
					  conv.f = g_yuzhi_pt;
            uint32_t ieee1 = conv.u;
            resp[0] = (ieee1 >> 24) & 0xFF;
            resp[1] = (ieee1 >> 16) & 0xFF;
            resp[2] = (ieee1 >> 8) & 0xFF;
            resp[3] = ieee1 & 0xFF;
            *resp_len = 4;
            break;
				}
        case 0x11: case 0x12: case 0x13:  // 写入阈值
						{ // 写入 CH 阈值
            if (content_len >= 4) {
                // 大端序拼成 uint32_t
                uint32_t ieee = ((uint32_t)content[0] << 24) |
                                ((uint32_t)content[1] << 16) |
                                ((uint32_t)content[2] << 8)  |
                                 (uint32_t)content[3];
                Float_IEEE754 conv;
                conv.u = ieee;
                float new_val = conv.f;

                 if (cmd == 0x11) {  // 设置 CH0 阈值
                    g_yuzhi_ch0 = new_val;
                    ch0_thresh = new_val;        // ? 同步到存储变量
                    flash_save_ch0_thresh();     // ?保存到Flash
                   
                }
                else if (cmd == 0x12) {  // 设置 CH1 阈值
                    g_yuzhi_ch1 = new_val;
                    ch1_thresh = new_val;        // 同步到存储变量
                    flash_save_ch1_thresh();     // 保存到Flash
                }
                else {  // cmd == 0x13 设置 CH2 阈值
                    g_yuzhi_pt = new_val;
                    
                }

                resp[0] = 0xFF;   // OK
                *resp_len = 1;
            } else {
                resp[0] = 0x02;   // 长度错误
                *resp_len = 1;
            }
            break;
        }
        default:
						send_error_frame();
            break;
    }
}

void handle_upgrade_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//系统升级类05
                      uint8_t *resp, uint8_t *resp_len) {
											    switch (cmd) {
        case 0x01:  // 升级请求
            resp[0] = 0xFF;  // 应答：即将升级
            *resp_len = 1;
				set_upgrade_flag();
						// 操作升级
            break;
        case 0x02:  // 准备传输
            resp[0] = 0x02;   // 返回错误或忽略
            *resp_len = 1;
            break;

        case 0x03:  // 执行升级流程
            resp[0] = 0xFF;  // 应答：即将升级
            *resp_len = 1;
						// 操作升级
            break;
        default:
				send_error_frame();
            break;
			}		
		}											
													
									
void handle_alarm_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,//告警与日志类06
                      uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:  // 是否主动上报告警
            
				if (content_len >= 1) {
                uint8_t mode = content[0];
                if (mode == 1 || mode == 2) {
                    g_alarm_enable = mode;
									  alarm_report_mode = mode;  // 同步到FLASH模块的变量
                    flash_save_alarm_mode();   // 保存到FLASH
                    resp[0] = 0xFF;   // OK
                    *resp_len = 1;
                } else {
                    resp[0] = 0x02;   // 参数错误
                    *resp_len = 1;
                }
            } else {
                resp[0] = 0x01;       // 长度错误
                *resp_len = 1;
            }
            break;

        case 0x02:  // 查询告警记录
					  
            //find_alarm_records();
						g_chaxun_enable = 0;
            break;
        case 0x03:  // 清除告警
					  clear_alarm_records();
            alarm_clear = 1;
				resp[0] = 0xFF;
            *resp_len = 1;
            break;
        case 0x04:  // 查询操作日志
            break;
        case 0x05:  // 清除操作日志
            break;
        default:
					send_error_frame();
            break;
    }
}
// 在 RTC.h 或 Function.c 顶部添加
static inline uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static inline uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/* 设置升级标志（只写Flash，不复位） */
static void set_upgrade_flag(void)
{
     static uint8_t buf[APP_CONFIG_SECTOR_SIZE];
    
    // 1. 读取整个扇区（4KB）的数据到内存，保护其他配置不被丢失
    for (uint16_t i = 0; i < APP_CONFIG_SECTOR_SIZE; i++) {
        buf[i] = internal_flash_read_Char(BOOT_CONFIG_ADDR + i);
    }

    // 2. 将 buf 的开头强制转换为 BootParam_t 指针，修改升级标志
    // （因为 BootParam 正好是参数区结构体的第一个成员）
    BootParam_t *pBootParam = (BootParam_t *)buf;
    pBootParam->updateFlag   = 0x5A;
    pBootParam->updateStatus = 0x01;

    // 3. 擦除 Flash 参数区（整块擦除）
    internal_flash_erase(BOOT_CONFIG_ADDR);
    
    // 4. 将修改后的完整 4KB 数据写回 Flash
    internal_flash_write_str_Char(BOOT_CONFIG_ADDR, buf, APP_CONFIG_SECTOR_SIZE);
}
/**
 * @brief  错误帧（类型0xFF, 命令字0xEEEE，内容为空）
 */
static void send_error_frame(void) {
    uint8_t empty = 0;   // 内容长度0
    uint8_t resp_bin[256];
    uint16_t frame_len = comm_build_bin_frame(
        g_device_id,
        FRAME_TYPE_ERROR,    // 0xFF
        0xEEEE,              // 固定命令字
        &empty,              // 无内容
        0,
        resp_bin
    );
    uint8_t ascii_out[512];
    uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
    USART1_SendData(ascii_out, ascii_len);
}

/**
 * @brief  发送上电/复位后的主动心跳帧（类型0x05, 命令字0x8888，内容为空）
 */
static void send_heartbeat(void)
{
	
	
    uint8_t heart_resp[] = { };          // 内容为空
    uint8_t resp_bin[256];
    uint16_t frame_len = comm_build_bin_frame(
        g_device_id,
        FRAME_TYPE_HEART,               // 0x05
        0x8888,                         // 主动心跳命令字
        heart_resp,
        sizeof(heart_resp),             // 长度为0
        resp_bin
    );
    uint8_t ascii_out[512];
    uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
    USART1_SendData(ascii_out, ascii_len);
}

float pt100_lookup_temp(float R) {
    int i;
    for (i = 0; i < TABLE_SIZE-1; i++) {
        if (R <= pt100_table[i+1][1]) {
            float R0 = pt100_table[i][1];
            float R1 = pt100_table[i+1][1];
            float t0 = pt100_table[i][0];
            float t1 = pt100_table[i+1][0];
            return t0 + (R - R0) * (t1 - t0) / (R1 - R0);
        }
    }
    return pt100_table[TABLE_SIZE-1][0]; // 超出范围
}