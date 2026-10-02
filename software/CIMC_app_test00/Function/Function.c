/************************************************************
 * 版权：2025CIMC Copyright。 
 * 文件：Function.c
 * 作者: Lingyu Meng
 * 平台: 2025CIMC IHD-V04
 * 版本: Lingyu Meng     2025/2/16     V0.01    original
************************************************************/


/************************* 头文件 *************************/

#include "Function.h"

/************************* 宏定义 *************************/
//#define MY_DEVICE_ID 0x0001
#define ALARM_LOG_MAX 10
#define APP_CONFIG_SECTOR_SIZE 4096  // Flash 擦除的最小单元通常是 4KB，请根据您的MCU手册确认
/************************ 变量定义 ************************/
		uint8_t *rx_buf;
		uint16_t rx_len;
    uint8_t bin_frame[256];
    uint16_t bin_len = 0;
		uint16_t g_device_id = 0x0001;    // 全局设备ID，默认 0x0001
		uint32_t g_firmware = 0x00000001;  // 当前固件版本
		uint32_t g_device_time = 0x6A09A79F;  // 当前时间戳（初始值示例）
		uint8_t g_bt_index = 20;  // 默认115200对应索引14,波特率索引
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
		uint8_t g_alarm_enable = 0;     // 01=主动上报, 02=不主动上报
		uint8_t g_chaxun_enable = 0;     // 1=查询报警记录 0=不查询报警记录
		uint8_t alarm_clear = 0;     // 1=清除报警记录 0=不清除报警记录		
/************************ 函数定义 ************************/

// 预留固件魔术字校验接口（默认为通过）
__weak int check_firmware_magic(void) {
    return 1;   // 1: 校验通过, 0: 失败
}

// 设置升级标志并复位
static void set_upgrade_flag_and_reset(void);

void set_upgrade_flag(void);
/************************************************************ 
 * Function :       System_Init
 * Comment  :       用于初始化MCU
 * Parameter:       null
 * Return   :       null
 * Author   :       Lingyu Meng
 * Date     :       2025-02-30 V0.1 original
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
	USART1_Config();
	LED_Init();
	
}
/************************************************************ 
 * Function :       Init_LED_Stat
 * Comment  :       系统初始化时用LED显示状态
 * Parameter:       null
 * Return   :       null
 * Author   :       Lingyu Meng
 * Date     :       2025-03-10 V0.1 original
************************************************************/


/************************************************************ 
 * Function :       UsrFunction
 * Comment  :       用户程序功能: LED1闪烁
 * Parameter:       null
 * Return   :       null
 * Author   :       Lingyu Meng
 * Date     :       2025-02-30 V0.1 original
************************************************************/

void UsrFunction(void) {
   LED1_OFF(); 
		LED3_ON();
	while (1) {
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

                if (dev_id == g_device_id || dev_id == 0xFFFF) {
                    if (frm_type == FRAME_TYPE_CMD) {
                        uint8_t  resp_content[64];				//回复内容
                        uint8_t  resp_content_len = 0;   // 回复保温长度
												uint8_t  resp_type = FRAME_TYPE_RESP;  // 默认正常应答
											  uint8_t module = (cmd_word >> 8) & 0xFF;   // 高字节：功能模块
                        uint8_t cmd    = cmd_word & 0xFF;          // 低字节：具体指令

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
                                resp_content[0] = 0x01;
                                resp_content_len = 1;
																resp_type = FRAME_TYPE_ERROR;
                                break;
                        }

                        uint8_t resp_bin[256];
                        uint16_t frame_len = comm_build_bin_frame(    // 返回值用新变量名
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
															delay_1ms(300); 
															NVIC_SystemReset(); 
													}
												if (frm_type == FRAME_TYPE_CMD && cmd_word == 0x01A2 && resp_type == FRAME_TYPE_RESP && resp_content_len == 1 && resp_content[0] == 0xFF) {
														// 短暂延时确保字节完全发出（TC 已在 SendData 中保证，但可加保险）
														delay_1ms(300);
														//NVIC_SystemReset();   // 系统复位，重启后采用默认波特率（Flash 未实现时）
												}

                    } else if (frm_type == FRAME_TYPE_HEART) {
                        uint8_t heart_resp[] = { };
                        uint8_t resp_bin[256];
                        uint16_t frame_len = comm_build_bin_frame(
                            g_device_id,
                            FRAME_TYPE_HEART,
                            0x8888,
                            heart_resp,
                            sizeof(heart_resp),
                            resp_bin
                        );
                        uint8_t ascii_out[512];
                        uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
                        USART1_SendData(ascii_out, ascii_len);
                    }
                }
            }
            USART1_ClearCommand();
        }
				// 定时上报逻辑
		if (g_report_enable) {
				uint32_t interval_ms;
				if (g_report_interval == 1)      interval_ms = 1000;
				else if (g_report_interval == 2) interval_ms = 3000;
				else                             interval_ms = 5000;

				if (sys_tick - last_report_tick >= interval_ms) {
						last_report_tick = sys_tick;

						// 构造上报数据（与首次应答相同的结构）
						uint8_t data[12];
						// 时间戳（若 RTC 可用，应读取实际值；此处暂时用 g_device_time）
						data[0] = (g_device_time >> 24) & 0xFF;
						data[1] = (g_device_time >> 16) & 0xFF;
						data[2] = (g_device_time >> 8) & 0xFF;
						data[3] = g_device_time & 0xFF;

						// CH0（示例值，实际替换）
						float ch0 = 21.59f * g_ch0_ratio;
						Float_IEEE754 c0; c0.f = ch0;
						uint32_t i0 = c0.u;
						data[4] = (i0 >> 24) & 0xFF;
						data[5] = (i0 >> 16) & 0xFF;
						data[6] = (i0 >> 8) & 0xFF;
						data[7] = i0 & 0xFF;

						// CH1
						float ch1 = 21.59f * g_ch1_ratio;
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
				}
		}
    }
		
}

void handle_system_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                       uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:  // 设备重启
            resp[0] = 0xFF;  // 应答：即将重启
            *resp_len = 1;
						// 操作重启
            break;
				case 0x02:  // 回复出厂设置
            resp[0] = 0xFF;  
            *resp_len = 1;
						// 操作重启
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
            // content 中应为时间数据，解析后设置 RTC
				uint32_t new_time = ((uint32_t)content[0] << 24 | (uint32_t)content[1] << 16 |(uint32_t)content[2] << 8 |(uint32_t)content[3]);
            g_device_time = new_time;   // 更新全局ID
            resp[0] = 0xFF;         // 成功
            *resp_len = 1;
            break;}
        case 0x06:  // 查询设备时间
            // 从 RTC 读取时间，填入 resp
						resp[0] = (g_device_time >> 24) & 0xFF;
						resp[1] = (g_device_time >> 16) & 0xFF;
						resp[2] = (g_device_time >> 8) & 0xFF;
						resp[3] = g_device_time & 0xFF;
            *resp_len = 4;
            break;
        case 0xA1:{  // 设置设备ID
            // content[0] content[1] 为新ID（大端）
				 uint16_t new_id = ((uint16_t)content[0] << 8) | content[1];
        if (new_id >= 0x0001 && new_id <= 0xFFFE) {
            g_device_id = new_id;   // 更新全局ID
            resp[0] = 0xFF;         // 成功
            *resp_len = 1;
        } else {
            resp[0] = 0x02;         // 参数非法
            *resp_len = 1;
        }
            resp[0] = 0xFF;
            *resp_len = 1;
            break;}
        case 0xA2: { // 设置波特率
            uint8_t index = content[0];
        if (index >= 17 && index <= 20) {
            g_bt_index = index;
            // TODO: 写入 Flash 持久化
            resp[0] = 0xFF;   // OK
            *resp_len = 1;
        } else {
            resp[0] = 0x02;   // 无效参数
            *resp_len = 1;
        }

            break;}
        case 0x11:  // 查询设备ID
            resp[0] = (g_device_id >> 8) & 0xFF;
            resp[1] = g_device_id & 0xFF;
            *resp_len = 2;
            break;
        case 0x12:  // 查询波特率
						resp[0] = g_bt_index;
            *resp_len = 1;
            break;
        default:
            resp[0] = 0x01;  // 未知命令
            *resp_len = 1;
            break;
    }
}
											 
void handle_data_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                     uint8_t *resp, uint8_t *resp_len) {
    			float adc0 = 21.59;  // 示例值
			float adc1 = 21.59;
			float pt100 = 21.59;  // 示例 1000
			      // 4. 转为 IEEE 754
            Float_IEEE754 conv0;
            conv0.f = adc0;
            uint32_t ieee0 = conv0.u;
			      Float_IEEE754 conv1;
            conv1.f = adc1;
            uint32_t ieee1 = conv1.u;
			      Float_IEEE754 convpt;
            convpt.f = pt100;
            uint32_t ieeept = convpt.u;
		switch (cmd) {
        case 0x01:  // 查询 CH0（滑动变阻器 ADC）
            // 读取 ADC 值，假设为 12 位
            resp[0] = (ieee0 >> 24) & 0xFF;
            resp[1] = (ieee0 >> 16) & 0xFF;
            resp[2] = (ieee0 >> 8) & 0xFF;
            resp[3] = ieee0 & 0xFF;
            *resp_len = 4;
            break;
        case 0x02:  // 查询 CH1（DAC 回读）
            
            resp[0] = (ieee1 >> 24) & 0xFF;
            resp[1] = (ieee1 >> 16) & 0xFF;
            resp[2] = (ieee1 >> 8) & 0xFF;
            resp[3] = ieee1 & 0xFF;
            *resp_len = 4;
            break;
        case 0x21:  // 查询外部 ADC（PT100）
            
            resp[0] = (ieeept >> 24) & 0xFF;
            resp[1] = (ieeept >> 16) & 0xFF;
            resp[2] = (ieeept >> 8) & 0xFF;
            resp[3] = ieeept & 0xFF;
            *resp_len = 4;
            break;
        case 0x41:  // 设置 CH0 变比
					{   // 设置CH0变比
            if (content_len >= 4) {
                // 1. 将下发的4字节大端序组合为 uint32_t
                uint32_t ieee = ((uint32_t)content[0] << 24) |
                                ((uint32_t)content[1] << 16) |
                                ((uint32_t)content[2] << 8)  |
                                 (uint32_t)content[3];
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
            resp[0] = 0x01;
            *resp_len = 1;
            break;
    }
}
										 
void handle_control_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                        uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:{  // 设置 DAC 输出电压
    if (content_len >= 2) {
        uint16_t dac_value = ((uint16_t)content[0] << 8) | content[1];
        if (dac_value <= 0x0FFF) {
            // 设置 DAC 输出（假设使用 DAC0，通道 0）

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
					float ch0_val = 21.59f * g_ch0_ratio;    // 示例，实际替换为 ADC 读取
					Float_IEEE754 conv0; conv0.f = ch0_val;
					uint32_t ieee0 = conv0.u;
					resp[4] = (ieee0 >> 24) & 0xFF;
					resp[5] = (ieee0 >> 16) & 0xFF;
					resp[6] = (ieee0 >> 8) & 0xFF;
					resp[7] = ieee0 & 0xFF;

					// 获取 CH1 数据并乘以变比（DAC 回读，暂时用固定值）
					float ch1_val = 21.59f * g_ch1_ratio;
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
					
            // 进入低功耗模式
            break;
        default:
            resp[0] = 0x01;
            *resp_len = 1;
            break;
    }
}
												
void handle_config_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
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

                if (cmd == 0x11)      g_yuzhi_ch0 = new_val;
                else if (cmd == 0x12) g_yuzhi_ch1 = new_val;
                else                  g_yuzhi_pt = new_val;

                resp[0] = 0xFF;   // OK
                *resp_len = 1;
            } else {
                resp[0] = 0x02;   // 长度错误
                *resp_len = 1;
            }
            break;
        }
        default:
            resp[0] = 0x01;
            *resp_len = 1;
            break;
    }
}

void handle_upgrade_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                      uint8_t *resp, uint8_t *resp_len) {
											    switch (cmd) {
        case 0x01:  // 升级请求
            resp[0] = 0xFF;  // 应答：即将升级
            *resp_len = 1;
						set_upgrade_flag();
            break;
        case 0x02:  // 准备传输(此指令应发给Bootloader，App中不处理)
            resp[0] = 0x02;   // 返回错误或忽略
            *resp_len = 1;
            break;

        case 0x03:  // 执行升级流程
            resp[0] = 0xFF;  // 应答：即将升级
            *resp_len = 1;
						// 操作升级
            break;
        default:
            resp[0] = 0x01;
            *resp_len = 1;
            break;
			}		
		}											
													
									
void handle_alarm_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                      uint8_t *resp, uint8_t *resp_len) {
    switch (cmd) {
        case 0x01:  // 是否主动上报告警
            
				if (content_len >= 1) {
                uint8_t mode = content[0];
                if (mode == 1 || mode == 2) {
                    g_alarm_enable = mode;
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
						g_chaxun_enable = 1;
            break;
        case 0x03:  // 清除告警
            alarm_clear = 1;
				resp[0] = 0xFF;
            *resp_len = 1;
            break;
        case 0x04:  // 查询操作日志
            break;
        case 0x05:  // 清除操作日志
            break;
        default:
            resp[0] = 0x01;
            *resp_len = 1;
            break;
    }
}
											/* 设置升级标志并软复位 */
/* 设置升级标志（不再在此函数复位） */
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
/****************************End*****************************/

