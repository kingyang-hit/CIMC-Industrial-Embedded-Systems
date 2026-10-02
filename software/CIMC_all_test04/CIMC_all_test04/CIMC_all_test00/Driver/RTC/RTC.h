#ifndef __RTC_H
#define __RTC_H
#include "HeaderFiles.h"

//!RTC闹铃触发时间 3s(BCD)
#define Alarm_Time 0x10

void RTC_Init(void);
void rtc_setup(void);
void rtc_show_time(void);
void rtc_show_alarm(void);
uint8_t usart_input_threshold(uint32_t value);
void rtc_pre_config(void);
void usart_read_line(char *buffer, uint32_t max_len);
extern uint8_t rtc_config_pending;   // 供 USART1.c 判断是否处于配置模式
void PMU_rtc_init(void);
// RTC.h 中添加
int rtc_set_timestamp(uint32_t timestamp);
uint32_t rtc_get_current_timestamp(void);
uint32_t rtc_query_timestamp_cmd(void);
int timestamp_to_rtc_time(uint32_t timestamp, rtc_parameter_struct* rtc_time);

#endif
