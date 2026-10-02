#include "RTC.h"
#include "USART1.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#define RTC_CLOCK_SOURCE_LXTAL
#define BKP_VALUE    0x32F0

extern uint32_t g_device_time ;

rtc_parameter_struct rtc_initpara;
rtc_alarm_struct rtc_alarm;
__IO uint32_t prescaler_a = 0, prescaler_s = 0;
uint32_t RTCSRC_FLAG = 0;

/* RTC 配置状态机 */
uint8_t rtc_config_pending = 0; 

/* 内部解析函数声明 */
static int rtc_parse_time_string(const char *time_str);
/* 如果 date_to_days 定义在其他文件，需要添加外部声明 */
extern uint32_t date_to_days(uint16_t year, uint8_t month, uint8_t day);
/*!
    \brief      从硬件RTC读取当前时间，并同步到全局时间戳变量
*/
void rtc_sync_timestamp(void)
{
    g_device_time = rtc_get_current_timestamp();
}

/*!
    \brief      main function
*/
void RTC_Init(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    
    rtc_pre_config();
    RTCSRC_FLAG = GET_BITS(RCU_BDCTL, 8, 9);

    if((BKP_VALUE != RTC_BKP0) || (0x00 == RTCSRC_FLAG)){
        rtc_setup();   // 未配置：提示用户输入时间
    }else{
        if (RESET != rcu_flag_get(RCU_FLAG_PORRST)){
            
        }else if (RESET != rcu_flag_get(RCU_FLAG_EPRST)){
           
        }

        // 重新初始化 RTC，确保预分频生效且时间不变
       // rtc_current_time_get(&rtc_initpara);    
				//rtc_initpara.factor_asyn = 0x7F;   // 与 rtc_pre_config 中 prescaler_a 一致
       // rtc_initpara.factor_syn = 0xFF;   // 与 rtc_pre_config 中 prescaler_s 一致
       // rtc_init(&rtc_initpara);               
        //RTC_BKP0 = BKP_VALUE;                  

        
        rtc_show_time();
        rtc_sync_timestamp(); 
    }
    rcu_all_reset_flag_clear();
}

// PMU_rtc_init 保持不变...
void PMU_rtc_init(void)
{
	rtc_parameter_struct   rtc_initpara;
	rtc_alarm_struct  rtc_alarm;

	/* enable PMU clocks */
	rcu_periph_clock_enable(RCU_PMU);
	/* allow access to BKP domain */
	pmu_backup_write_enable();
	/* reset backup domain */
	rcu_bkp_reset_enable();
	rcu_bkp_reset_disable();

	/* enable RCU_LXTAL */
	rcu_osci_on(RCU_LXTAL);
	/* wait till RCU_LXTAL is ready */
	rcu_osci_stab_wait(RCU_LXTAL);
	/* select RCU_LXTAL as RTC clock source */
	rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
	/* enable RTC Clock */
	rcu_periph_clock_enable(RCU_RTC);
	
	   // ★ 修复：使用与主RTC相同的预分频值（32.768kHz）
    rtc_initpara.factor_asyn = 0x7F;   // 改为 127
    rtc_initpara.factor_syn = 0xFF;    // 改为 255
    

	//!分频位1HZ
	//rtc_initpara.factor_asyn = 0x63;
	//rtc_initpara.factor_syn = 0x13F;
	rtc_initpara.year = 0x16;
	rtc_initpara.day_of_week = RTC_SATURDAY;
	rtc_initpara.month = RTC_APR;
	rtc_initpara.date = 0x30;
	rtc_initpara.display_format = RTC_24HOUR;
	rtc_initpara.am_pm = RTC_AM;

	/* configure current time */
	rtc_initpara.hour = 0x00;
	rtc_initpara.minute = 0x00;
	rtc_initpara.second = 0x00;
	rtc_init(&rtc_initpara);
	rtc_alarm_disable(RTC_ALARM0);
	rtc_alarm.alarm_mask = RTC_ALARM_DATE_MASK | RTC_ALARM_HOUR_MASK | RTC_ALARM_MINUTE_MASK;
	rtc_alarm.weekday_or_date = RTC_ALARM_DATE_SELECTED;
	rtc_alarm.alarm_day = 0x31;
	rtc_alarm.am_pm = RTC_AM;

	/* RTC alarm value */
	rtc_alarm.alarm_hour = 0x00;
	rtc_alarm.alarm_minute = 0x00;
	rtc_alarm.alarm_second =0x10;	//!Alarm_Time 秒后 触发闹钟
	

	/* configure RTC alarm */
	rtc_alarm_config(RTC_ALARM0 , &rtc_alarm);

	rtc_interrupt_enable(RTC_INT_ALARM0);
	rtc_alarm_enable(RTC_ALARM0);

	rtc_flag_clear(RTC_FLAG_ALRM0);
      /* 配置 EXTI 线 17 (RTC Alarm) */
    exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_17);

    /* 配置 NVIC 优先级并使能 RTC 闹钟中断 */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(RTC_Alarm_IRQn, 0U, 0U);

}

void rtc_pre_config(void)
{
    #if defined (RTC_CLOCK_SOURCE_IRC32K)
          rcu_osci_on(RCU_IRC32K);
          rcu_osci_stab_wait(RCU_IRC32K);
          rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
          prescaler_s = 0x13F;
          prescaler_a = 0x63;
    #elif defined (RTC_CLOCK_SOURCE_LXTAL)
          rcu_osci_on(RCU_LXTAL);
          rcu_osci_stab_wait(RCU_LXTAL);
          rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);
          prescaler_s = 0xFF;
          prescaler_a = 0x7F;
    #else
    #error RTC clock source should be defined.
    #endif
    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}

void rtc_setup(void)
{
    rtc_config_pending = 1;
    
}

/*!
    \brief      解析时间字符串并写入 RTC
    \note       现在支持直接传入时间字符串，并使用全局时间戳进行赋值
*/
static int rtc_parse_time_string(const char *time_str)
{
    uint32_t year, month, day, hour, min, sec;

    // 尝试两种分隔符
    if (sscanf(time_str, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &min, &sec) != 6) {
        if (sscanf(time_str, "%d-%d-%d %d-%d-%d", &year, &month, &day, &hour, &min, &sec) != 6) {
            return -1; // 格式错误
        }
    }

    if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour > 23 || min > 59 || sec > 59) {
        return -2; // 数值超限
    }

    // 转 BCD 码
    rtc_initpara.year   = ((year - 2000) / 10) << 4 | ((year - 2000) % 10);
    rtc_initpara.month  = ((month / 10) << 4) | (month % 10);
    rtc_initpara.date   = ((day / 10) << 4) | (day % 10);
    rtc_initpara.hour   = ((hour / 10) << 4) | (hour % 10);
    rtc_initpara.minute = ((min / 10) << 4) | (min % 10);
    rtc_initpara.second = ((sec / 10) << 4) | (sec % 10);
    rtc_initpara.day_of_week = RTC_SUNDAY; // 可优化为根据日期自动计算
    rtc_initpara.display_format = RTC_24HOUR;
    rtc_initpara.am_pm = RTC_AM;
		
		/* ★★★ 必须添加：设置正确的预分频值 ★★★ */
    rtc_initpara.factor_asyn = 0x7F;   // 127
    rtc_initpara.factor_syn = 0xFF;    // 255

    if (rtc_init(&rtc_initpara) == ERROR) {
        return -3; // RTC 写入失败
    }
    
    RTC_BKP0 = BKP_VALUE;
    
    // ★ 关键：设置成功后，立即同步全局时间戳变量 ★
    rtc_sync_timestamp(); 
    
    return 0;
}

/*!
    \brief      查询并输出当前的 UTC 时间戳
*/
void rtc_query_timestamp(void)
{
    // 每次查询前，先从硬件刷新一下时间戳
    rtc_sync_timestamp(); 
   
}

/*!
    \brief      RTC 命令处理（在 usart_recv_task 中调用）
*/
void rtc_process_command(void)
{
    char *cmd = (char*)recv_real_buf;

    // 去除尾部换行符
    for (int i = strlen(cmd) - 1; i >= 0; i--) {
        if (cmd[i] == '\r' || cmd[i] == '\n') cmd[i] = '\0';
    }

    // 新增：查询时间戳指令
    if (strcmp(cmd, "RTC Query") == 0) {
        rtc_query_timestamp();
        return;
    }

    if (rtc_config_pending == 0) {
        // 等待 "RTC Config" 指令
        if (strcmp(cmd, "RTC Config") == 0) {
            rtc_config_pending = 1;
            
        }
    } else {
        // 已进入配置模式，尝试解析时间字符串
        int ret = rtc_parse_time_string(cmd);
        if (ret == 0) {
            
            rtc_show_time();
            rtc_config_pending = 0;
        } else if (ret == -1) {
            
        } else if (ret == -2) {
            
            rtc_config_pending = 0;   
        } else if (ret == -3) {
            
            rtc_config_pending = 0;
        }
    }
}

void rtc_show_time(void)
{
    unsigned char time_str[32] = {0};
    rtc_current_time_get(&rtc_initpara);
    
    snprintf((char*)time_str, sizeof(time_str), 
            "%02d:%02d:%02d", 
            rtc_initpara.hour, rtc_initpara.minute, rtc_initpara.second);
}

void rtc_show_alarm(void)
{
    rtc_alarm_get(RTC_ALARM0, &rtc_alarm);
    printf("The alarm: %0.2x:%0.2x:%0.2x \n\r", rtc_alarm.alarm_hour, rtc_alarm.alarm_minute,
           rtc_alarm.alarm_second);
}




// 添加这些辅助函数到 RTC.c
static const uint16_t month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static uint8_t is_leap_year_calc(uint16_t year) {
    return ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
}

// 手动将时间戳转换为年月日时分秒
static void  timestamp_to_datetime(uint32_t timestamp, uint16_t *year, uint8_t *month, 
                                   uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second) {
    uint32_t days = timestamp / 86400;
    uint32_t seconds_remain = timestamp % 86400;
    
    // 计算时分秒
    *hour = seconds_remain / 3600;
    *minute = (seconds_remain % 3600) / 60;
    *second = seconds_remain % 60;
    
    // 计算年月日（从1970-01-01开始）
    *year = 1970;
    uint32_t day_of_year = days;
    
    while (1) {
        uint16_t days_in_year = is_leap_year_calc(*year) ? 366 : 365;
        if (day_of_year < days_in_year) {
            break;
        }
        day_of_year -= days_in_year;
        (*year)++;
        if (*year > 2099) {
            *year = 2000;
            break;
        }
    }
    
    // 计算月份和日期
    *month = 1;
    while (1) {
        uint8_t days_in_month = month_days[*month - 1];
        if (*month == 2 && is_leap_year_calc(*year)) {
            days_in_month = 29;
        }
        if (day_of_year < days_in_month) {
            break;
        }
        day_of_year -= days_in_month;
        (*month)++;
    }
    *day = day_of_year + 1;
}

int timestamp_to_rtc_time(uint32_t timestamp, rtc_parameter_struct* rtc_time)
{
    uint16_t year;
    uint8_t month, day, hour, minute, second;
    
    // 使用手动转换
    timestamp_to_datetime(timestamp, &year, &month, &day, &hour, &minute, &second);
    
    // 检查年份范围
    if (year < 2000 || year > 2099) {
        
        return -2;
    }
    
    // 转换为BCD格式
    rtc_time->year = ((year - 2000) / 10) << 4 | ((year - 2000) % 10);
    rtc_time->month = (month / 10) << 4 | (month % 10);
    rtc_time->date = (day / 10) << 4 | (day % 10);
    rtc_time->hour = (hour / 10) << 4 | (hour % 10);
    rtc_time->minute = (minute / 10) << 4 | (minute % 10);
    rtc_time->second = (second / 10) << 4 | (second % 10);
    rtc_time->day_of_week = RTC_SUNDAY;
    rtc_time->display_format = RTC_24HOUR;
    rtc_time->am_pm = RTC_AM;
		rtc_time->factor_asyn = 0x7F;   // 127
    rtc_time->factor_syn = 0xFF;    // 255
    
    return 0;
}
int rtc_set_timestamp(uint32_t timestamp)
{
    rtc_parameter_struct rtc_time;
    
    // 转换时间戳
    if (timestamp_to_rtc_time(timestamp, &rtc_time) != 0) {
        return -1;
    }
    
    // 使能备份域写访问
    pmu_backup_write_enable();
    
    // 初始化RTC
    if (rtc_init(&rtc_time) == ERROR) {
        return -2;
    }
    
    // 保存备份寄存器
    RTC_BKP0 = BKP_VALUE;
    
    // 同步全局时间戳
    g_device_time = timestamp;
    
    // 显示设置后的时间

    
    return 0;
}
uint32_t rtc_get_current_timestamp(void)
{
    rtc_parameter_struct rtc_time;

    
    /* 读取当前RTC时间 */
    rtc_current_time_get(&rtc_time);
    
    /* BCD转十进制 */
    uint16_t year  = ((rtc_time.year >> 4) * 10 + (rtc_time.year & 0x0F)) + 2000;
    uint8_t  month = ((rtc_time.month >> 4) * 10 + (rtc_time.month & 0x0F));
    uint8_t  day   = ((rtc_time.date >> 4) * 10 + (rtc_time.date & 0x0F));
    uint8_t  hour  = ((rtc_time.hour >> 4) * 10 + (rtc_time.hour & 0x0F));
    uint8_t  min   = ((rtc_time.minute >> 4) * 10 + (rtc_time.minute & 0x0F));
    uint8_t  sec   = ((rtc_time.second >> 4) * 10 + (rtc_time.second & 0x0F));
    
    /* ★ 使用手动计算，不用 mktime，避免时区问题 */
    return date_to_days(year, month, day) * 86400UL + 
           hour * 3600UL + min * 60UL + sec;
}
uint32_t rtc_query_timestamp_cmd(void)
{
    uint32_t timestamp = rtc_get_current_timestamp();
    g_device_time = timestamp;
    return timestamp;
}
