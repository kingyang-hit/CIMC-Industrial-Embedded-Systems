/************************************************************
 * 版权：2025CIMC Copyright。 
 * 文件：Function.h
 * 作者: Lingyu Meng
 * 平台: 2025CIMC IHD-V04
 * 版本: Lingyu Meng     2025/2/16     V0.01    original
************************************************************/

#ifndef __FUNCTION_H
#define __FUNCTION_H

/************************* 头文件 *************************/

#include "HeaderFiles.h"

/************************* 宏定义 *************************/


/************************ 变量定义 ************************/
extern volatile uint8_t need_reset_for_baudrate;
typedef enum {
    SAMPLE_5S  = 5,
    SAMPLE_10S = 10,
    SAMPLE_15S = 15
} sample_interval_t;
// Function.h 中添加
//extern uint8_t baudrate_index;  // 波特率索引
/************************ 函数定义 ************************/

void System_Init(void);      	// 系统初始化
void UsrFunction(void);         // 用户函数
void Init_LED_Stat(void);		// 系统初始化时用LED显示状态
void update_oled_time(void) ;
void enter_start_mode(void) ;
void enter_stop_mode(void)  ;
 void load_cfg_from_flash(void) ;
static void save_cfg_to_flash(sample_interval_t interval);
 uint8_t delay_with_key_check(uint32_t ms);
uint8_t is_leap_year(uint16_t year) ;
uint8_t get_month_days(uint16_t year, uint8_t month) ;
uint32_t date_to_days(uint16_t year, uint8_t month, uint8_t day) ;
uint32_t rtc_to_unix_timestamp(rtc_parameter_struct rtc_initpara)  ;
void ad3344_process(void) ;
void ad3344_ExtRef(void) ;
void ad3344_sampling_task(void);
int8_t ReadRatioLine(uint8_t *tx_buf);
int8_t ReadLimitLine(uint8_t *tx_buf);

void handle_system_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                       uint8_t *resp, uint8_t *resp_len);
void handle_data_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                     uint8_t *resp, uint8_t *resp_len);
void handle_control_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                        uint8_t *resp, uint8_t *resp_len);
void handle_config_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                       uint8_t *resp, uint8_t *resp_len);
void handle_upgrade_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                      uint8_t *resp, uint8_t *resp_len);
void handle_alarm_cmd(uint8_t cmd, uint8_t *content, uint8_t content_len,
                      uint8_t *resp, uint8_t *resp_len);
void check_and_report_alarm(uint8_t channel, float threshold, float real_value);
static inline uint8_t bcd_to_dec(uint8_t bcd);
#endif
//int8_t ReadRatioLine(uint8_t *tx_buf);
//int8_t ReadLimitLine(uint8_t *tx_buf) ;

/****************************End*****************************/

