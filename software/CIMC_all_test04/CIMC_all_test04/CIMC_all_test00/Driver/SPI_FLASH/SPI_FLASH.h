#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

/************************* 头文件 *************************/

#include "HeaderFiles.h"

#define  SPI_FLASH_PAGE_SIZE           0x100
#define  SPI_FLASH_CS_LOW()            gpio_bit_reset(GPIOB, GPIO_PIN_12)
#define  SPI_FLASH_CS_HIGH()           gpio_bit_set(GPIOB, GPIO_PIN_12)

// SPI_FLASH.h 中添加
#define FLASH_SECTOR_SIZE       4096  // 4KB 扇区

// 为每个参数分配独立的扇区（避免互相干扰）
#define FLASH_SECTOR_DEVICE_ID   0     // 扇区0：设备ID

#define FLASH_SECTOR_BAUDRATE    1     // 扇区1：波特率+索引号  
#define FLASH_SECTOR_CH0_RATIO   2     // 扇区2：CH0变比
#define FLASH_SECTOR_CH1_RATIO   3     // 扇区3：CH1变比
#define FLASH_SECTOR_CH0_THRESH  4     // 扇区4：CH0阈值
#define FLASH_SECTOR_CH1_THRESH  5     // 扇区5：CH1阈值
#define FLASH_SECTOR_ALARM_MODE  6     // 扇区6：告警模式

// 对应的地址
#define FLASH_ADDR_DEVICE_ID     (FLASH_SECTOR_DEVICE_ID * FLASH_SECTOR_SIZE)

#define FLASH_ADDR_BAUDRATE      (FLASH_SECTOR_BAUDRATE * FLASH_SECTOR_SIZE)
#define FLASH_ADDR_CH0_RATIO     (FLASH_SECTOR_CH0_RATIO * FLASH_SECTOR_SIZE)
#define FLASH_ADDR_CH1_RATIO     (FLASH_SECTOR_CH1_RATIO * FLASH_SECTOR_SIZE)
#define FLASH_ADDR_CH0_THRESH    (FLASH_SECTOR_CH0_THRESH * FLASH_SECTOR_SIZE)
#define FLASH_ADDR_CH1_THRESH    (FLASH_SECTOR_CH1_THRESH * FLASH_SECTOR_SIZE)
#define FLASH_ADDR_ALARM_MODE    (FLASH_SECTOR_ALARM_MODE * FLASH_SECTOR_SIZE)

#define FLASH_BASE_ADDR        0x000000
#define SECTOR_SIZE            4096  // 4K Bytes per sector

// --- 告警日志区 (推荐：从 0x008000 开始，完全隔离) ---
#define ALARM_LOG_START_ADDR   0x008000  
#define ALARM_LOG_TOTAL_SIZE   (SECTOR_SIZE * 4) // 16KB
#define ALARM_LOG_END_ADDR     (ALARM_LOG_START_ADDR + ALARM_LOG_TOTAL_SIZE - 1)
// 波特率数据结构（索引+实际值）
typedef struct {
    uint8_t  index;      // 波特率索引  17-20
    uint32_t value;      // 实际波特率值
} Baudrate_Config_t;
// 波特率映射表结构体
typedef struct {
    uint8_t  index;
    uint32_t baudrate;
} Baudrate_Map_t;
// ========== 默认值 ==========
#define DEFAULT_DEVICE_ID          0x0001
#define DEFAULT_BAUDRATE_INDEX    20      // 默认115200对应索引20
#define DEFAULT_BAUDRATE_VALUE  115200
// 波特率映射表大小
#define BAUDRATE_MAP_SIZE         4
#define DEFAULT_CH0_RATIO          1.0f
#define DEFAULT_CH1_RATIO          1.0f
#define DEFAULT_CH0_THRESH         0.0f
#define DEFAULT_CH1_THRESH         0.0f
#define DEFAULT_ALARM_MODE       0x02        // 默认值：0x02(仅存储)
// ========== 全局变量 ==========
extern uint32_t device_id;

extern uint32_t  current_baudrate;
extern uint8_t   g_baudrate_index;      // 波特率索引（统一使用这个）

extern float    ch0_ratio;
extern float    ch1_ratio;
extern float    ch0_thresh;
extern float    ch1_thresh;
extern uint8_t alarm_report_mode; // 上报模式: 0x01主动上报, 0x02仅存储
extern uint32_t log_write_addr;   // 当前写入指针
extern uint16_t g_device_id;
// 告警日志结构体 (大小需对齐)
typedef struct {
    uint8_t  valid_flag;      // 0xFF表示无效/空，0x01表示有效
    uint8_t  channel;         // 通道号 (0=CH0, 1=CH1)
    uint16_t year;            // 年
    uint8_t  month, day;      // 月, 日
    uint8_t  hour, minute, second; // 时, 分, 秒
    float    threshold;       // 触发告警时的阈值
    float    real_value;      // 触发告警时的实际采集值
} Alarm_Log_TypeDef;

// ========== 接口函数 ==========
void flash_params_init(void);           // 上电初始化，读取所有参数
void flash_save_device_id(void);
void flash_save_baudrate(void);
void flash_save_ch0_ratio(void);
void flash_save_ch1_ratio(void);
void flash_save_ch0_thresh(void);
void flash_save_ch1_thresh(void);
void flash_save_alarm_mode(void); 
/* initialize SPI1 GPIO and parameter */
void spi_flash_init(void);
/* erase the specified flash sector */
void spi_flash_sector_erase(uint32_t sector_addr);
/* erase the entire flash */
void spi_flash_bulk_erase(void);
/* write more than one byte to the flash */
void spi_flash_page_write(uint8_t* pbuffer,uint32_t write_addr,uint16_t num_byte_to_write);
/* write block of data to the flash */
void spi_flash_buffer_write(uint8_t* pbuffer,uint32_t write_addr,uint16_t num_byte_to_write);
/* read a block of data from the flash */
void spi_flash_buffer_read(uint8_t* pbuffer,uint32_t read_addr,uint16_t num_byte_to_read);
/* read flash identification */
uint32_t spi_flash_read_id(void);
/* initiate a read data byte (read) sequence from the flash */
void spi_flash_start_read_sequence(uint32_t read_addr);
/* read a byte from the SPI flash */
uint8_t spi_flash_read_byte(void);
/* send a byte through the SPI interface and return the byte received from the SPI bus */
uint8_t spi_flash_send_byte(uint8_t byte);
/* send a half word through the SPI interface and return the half word received from the SPI bus */
uint16_t spi_flash_send_halfword(uint16_t half_word);
/* enable the write access to the flash */
void spi_flash_write_enable(void);
/* poll the status of the write in progress (wip) flag in the flash's status register */
void spi_flash_wait_for_write_end(void);


 void flash_sector_write(uint32_t addr, uint8_t *data, uint32_t len);
 uint32_t read_uint32_with_default(uint32_t addr, uint32_t default_val);
 uint8_t read_uint8_with_default(uint32_t addr, uint8_t default_val);
float read_float_with_default(uint32_t addr, float default_val);
// SPI_FLASH.h 中添加函数声明
void read_baudrate_from_flash(void);
void flash_save_baudrate_index(uint8_t index);
void flash_save_baudrate_value(uint32_t value);
void flash_save_baudrate_with_index(uint8_t index);
uint8_t baudrate_index_to_value(uint8_t index, uint32_t *out_value);
void flash_load_baudrate(void);
#endif
