/************************************************************
 * 版权：2025CIMC Copyright。
 * 文件：Function.c
 * 作者: Jialei Zhao
 * 平台: 2025CIMC IHD-V04
 * 版本: Jialei Zhao     2026/2/4     V0.01    original
************************************************************/


#include "Function.h"
#include <time.h>
#include "rom.h"
#include "bootloader.h"
#include "BootConfig.h"
#include "HeaderFiles.h"

/************************* 宏定义 *************************/
typedef uint8_t bool;
#define TRUE  1
#define FALSE 0

typedef void (*pFunction)(void);
pFunction jump2app;

#define CONFIG_SIZE        (1024 * 4)
#define CONFIG_APP_SIZE    (1024 * 20)

// 固件暂存区域（App 升级前存放新固件）
#define APP_DOWNLOAD_ADDR  0x08051000   // 128KB 暂存区

// 系统滴答计数（需在 SysTick_Handler 中递增）
uint32_t sys_tick =0;

/************************ 变量定义 ************************/
typedef struct __attribute__((packed)) Parameter_SUM {
    BootParam_t   BootParam;
    BootParam_t   BootParam_Reserved;
    UpdateLog_t   UpdateLog;
    UserConfig_t  UserConfig;
    CalibData_t   CalibData;
} Parameter_t;

Parameter_t my_param_sum = { 0 };
uint8_t config_buf[CONFIG_APP_SIZE] = { 0 };

// 固件升级接收相关
uint8_t  fw_buf[128 * 1024] = { 0 };    // 暂存缓冲区 128KB
uint32_t fw_len = 0;

// 设备 ID（与 App 保持一致）
uint16_t g_device_id = 0x0001;
float g_ch0_ratio = 1.0f;   // CH0 变比，默认 1.0
float g_ch1_ratio = 1.0f;   // CH1 变比，默认 1.0
		uint8_t g_alarm_enable = 0;     // 01=主动上报, 02=不主动上报
		float g_yuzhi_ch0 = 21.59f;   // 默认阈值示例
		float g_yuzhi_ch1 = 21.59f;
extern uint8_t  g_baudrate_index; //波特率索引
/************************ 函数声明 ************************/
static void Analysis_ConfigForAddr(void);
uint32_t crc32_calc(uint8_t* data, uint32_t len);
void mcu_software_reset(void);
void jump_to_app(void);
bool receive_and_upgrade_firmware(void);
bool perform_update(uint8_t *data, uint32_t size);
void send_error_response(uint16_t cmd_word, uint8_t err_code);
void update_boot_param_after_upgrade(void);
__weak int check_firmware_magic(void);
static void send_magic_error_frame(void);
/************************************************************
 * Function :       System_Init
 * Comment  :       初始化 MCU 基本外设
 ************************************************************/
void System_Init(void)
{
    systick_config();
    
    LED_Init();
		spi_flash_init();            //FLASH——SPI
		OLED_Init();
		OLED_Clear();
    OLED_ShowString(0, 0, "2026116659", 16);
		OLED_ShowString(0, 16, "Bootloader", 16);
    OLED_Refresh();
		USART1_Config();
}

/************************************************************
 * Function :       UsrFunction
 * Comment  :       Bootloader 主流程
 ************************************************************/
void UsrFunction(void)
{
    // 1. 读取参数区
    Analysis_ConfigForAddr();
    memcpy(&my_param_sum, config_buf, sizeof(Parameter_t));

    // 2. 如果参数区无效，自动初始化默认参数
    if (my_param_sum.BootParam.magicWord != 0x5AA5C33C) {
        memset(&my_param_sum, 0, sizeof(my_param_sum));
        my_param_sum.BootParam.magicWord    = 0x5AA5C33C;
        my_param_sum.BootParam.version      = 0x0001;
        my_param_sum.BootParam.structSize   = 256;
        my_param_sum.BootParam.appStartAddr = 0x08011000;
        my_param_sum.BootParam.appStackAddr = *(__IO uint32_t*)0x08011000;
        my_param_sum.BootParam.appEntryAddr = *(__IO uint32_t*)(0x08011000 + 4);
        my_param_sum.BootParam.updateFlag   = 0x00;
        my_param_sum.BootParam.updateStatus = 0x00;
        my_param_sum.BootParam.bootFailCount = 0;
        my_param_sum.BootParam.tailMagic    = 0xA5A5C3C3;

        memcpy(config_buf, &my_param_sum, sizeof(Parameter_t));
        internal_flash_erase(BOOT_CONFIG_ADDR);
        internal_flash_write_str_Char(BOOT_CONFIG_ADDR, config_buf, CONFIG_SIZE);
    }

    // 3. 强制更新 App 地址 (支持带魔术字偏移的解析)
    my_param_sum.BootParam.appStartAddr = 0x08011000;
    uint32_t app_addr = 0x08011000;
		uint32_t stack = *(volatile uint32_t*)app_addr;

    if( (stack & 0x2FF00000) == 0x20000000 ) {
        my_param_sum.BootParam.appStackAddr = stack;
        my_param_sum.BootParam.appEntryAddr = *(volatile uint32_t*)(app_addr + 4);
    } else {
        my_param_sum.BootParam.appStackAddr = 0x20003000;
        my_param_sum.BootParam.appEntryAddr = 0;
    }

    // 4. 判断是否有升级请求
    bool upgrade_requested = (my_param_sum.BootParam.updateFlag == 0x5A &&
                              my_param_sum.BootParam.updateStatus == 0x01);
    if (!upgrade_requested) {

        delay_1ms(5000);

        jump_to_app();

        while (1);
    }

    // ========== 有升级请求：打印信息，进入 10 秒等待 ==========
    my_printf("using command to interrupt start Application\r\n");
    my_printf("wait for start Application(10s)……\r\n");

    uint32_t start_tick = sys_tick;
    uint8_t second = 10;
    bool cmd_received = FALSE;

    // --- 倒计时循环：只负责监听0x0502 ---
    while (second > 0) {
        if (recv_flag) {
            uint8_t bin_frame[256];
            uint16_t bin_len = 0;
            if (comm_parse_ascii_frame(recv_real_buf, recv_real_len, bin_frame, &bin_len) == 0) {
                uint16_t dev_id   = (bin_frame[2] << 8) | bin_frame[3];
                uint8_t  frm_type = bin_frame[4];
                uint16_t cmd_word = (bin_frame[5] << 8) | bin_frame[6];

                // 检测到 0x0502 指令
                if ((dev_id == g_device_id || dev_id == 0xFFFF) &&
                    frm_type == FRAME_TYPE_CMD && cmd_word == 0x0502) {
                    
                    cmd_received = TRUE;
											                    uint8_t ok = 0xFF;
                    uint8_t resp_bin[256];
                    uint16_t frame_len = comm_build_bin_frame(g_device_id, FRAME_TYPE_RESP, 0x0502, &ok, 1, resp_bin);
                    uint8_t ascii_out[512];
                    uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
                    USART1_SendData(ascii_out, ascii_len);
                    delay_1ms(50); 
										USART1_ClearCommand();
                    break; // 收到0x0502，立刻停止倒计时，跳出循环！
                }
            }
            USART1_ClearCommand();
        }

        // 倒计时打印
        if (sys_tick - start_tick >= 1000) {
            start_tick = sys_tick;
            second--;
            if (second == 7 || second == 4 || second == 1) {
                my_printf("wait for start Application(%ds)……\r\n", second);
            }
        }
    }

    // --- 根据倒计时结果决定走向 ---
    if (!cmd_received) {
        // 10秒倒计时结束，没有收到0x0502，直接跳转 App
        jump_to_app();
    } else {
        // 收到0x0502，停止倒计时成功，进入无限等待接收 bin 文件流程
        bool success = receive_and_upgrade_firmware();
        
        if (success) {
            update_boot_param_after_upgrade();
        } else {
            send_error_response(0x0502, 0x01);
        }
        mcu_software_reset();
    }

    while (1);
}

/************************************************************
 * Function :       Analysis_ConfigForAddr
 ************************************************************/
static void Analysis_ConfigForAddr(void)
{
    for (uint16_t i = 0; i < 1024 * 4; i++) {
        config_buf[i] = internal_flash_read_Char(BOOT_CONFIG_ADDR + i);
    }
}

/************************************************************
 * Function :       crc32_calc
 ************************************************************/
uint32_t crc32_calc(uint8_t* data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

/************************************************************
 * Function :       mcu_software_reset
 ************************************************************/
void mcu_software_reset(void)
{
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

/************************************************************
 * Function :       iap_load_app (已修复偏移逻辑)
 ************************************************************/
void iap_load_app(uint32_t appxaddr)
{
    // 回归标准：栈顶在偏移0，入口在偏移4
    uint32_t app_stack_addr = appxaddr;
    uint32_t app_entry_addr = appxaddr + 4; 
    
    uint32_t actual_stack = *(__IO uint32_t*)app_stack_addr;
    
    if ((actual_stack & 0x2FF00000) == 0x20000000) {
        __disable_irq();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL  = 0;
        for (uint32_t i = 0; i < 8; i++) {
            NVIC->ICER[i] = 0xFFFFFFFF;
            NVIC->ICPR[i] = 0xFFFFFFFF;
        }
        __DSB();
        __ISB();

        SCB->VTOR = appxaddr; 
        __set_MSP(actual_stack); 
        
        jump2app = (pFunction)(*(__IO uint32_t*)(app_entry_addr));
        jump2app();

        while (1);
    } else {
        mcu_software_reset(); // 防卡死
    }
}

/************************************************************
 * Function :       jump_to_app
 ************************************************************/
void jump_to_app(void)
{
    __disable_irq();
    if ((my_param_sum.BootParam.appEntryAddr & 0xFF000000) == 0x08000000) {
        iap_load_app(my_param_sum.BootParam.appStartAddr);
    } else {
        mcu_software_reset();
    }
}

/************************************************************
 * 升级流程相关函数
 ************************************************************/

/**
 * @brief  接收纯 bin 二进制流，收到第一包立刻校验魔术字，收到 0x0503 结束
 */
/**
 * @brief  接收纯 bin 二进制流，收到第一包立刻校验魔术字，收到 0x0503 结束
 */
bool receive_and_upgrade_firmware(void)
{
	
    fw_len = 0;
    bool magic_checked = FALSE; 
    bool session_invalid = FALSE; // 标记当前接收会话是否已损坏
    bool stream_end = FALSE;      // 标记固件裸流是否发送完毕

    uint32_t last_recv_tick = sys_tick;

    while (1) {
        if (recv_flag) {
            last_recv_tick = sys_tick;

            if (!stream_end) {
                // ========== 阶段1：接收固件裸流 ==========
                // 不调用 comm_parse_ascii_frame，直接把收到的数据当作固件存入 fw_buf
                if (!session_invalid) {
                    if (fw_len + recv_real_len <= sizeof(fw_buf)) {
                        memcpy(&fw_buf[fw_len], recv_real_buf, recv_real_len);
                        fw_len += recv_real_len;
                        
                        if (!magic_checked && fw_len >= 4) {
                            magic_checked = TRUE; 
                            
                            if (fw_buf[0] == 0x5A && fw_buf[1] == 0xA5 &&
                                fw_buf[2] == 0xC3 && fw_buf[3] == 0x3C) {
                                // 魔术字正确，正常接收（无需在此处发应答，0x0502应答在进入前已发）
                            } else {
                                // 魔术字错误
                                send_magic_error_frame();
                                delay_1ms(100); 
                                
                                session_invalid = TRUE;
                                // 不清空 fw_buf，后续数据依然会进来，但不再存入，直接忽略
                            }
                        }
                    } else {
                        // 缓冲区溢出
                        session_invalid = TRUE;
                    }
                }
                // 如果 session_invalid 为 TRUE，后续收到的裸流数据直接丢弃
            } else {
                // ========== 阶段2：等待控制指令 ==========
                uint8_t bin_frame[256];
                uint16_t bin_len = 0;
                
                if (comm_parse_ascii_frame(recv_real_buf, recv_real_len, bin_frame, &bin_len) == 0) {
                    uint16_t dev_id   = (bin_frame[2] << 8) | bin_frame[3];
                    uint8_t  frm_type = bin_frame[4];
                    uint16_t cmd_word = (bin_frame[5] << 8) | bin_frame[6];

                    if ((dev_id == g_device_id || dev_id == 0xFFFF) && frm_type == FRAME_TYPE_CMD) {
                        if (cmd_word == 0x0503) {
                            if (!session_invalid) {
                                // 会话有效，收到结束指令，正常退出
                                uint8_t ok = 0xFF;
                                uint8_t resp_bin[256];
                                uint16_t frame_len = comm_build_bin_frame(g_device_id, FRAME_TYPE_RESP, 0x0503, &ok, 1, resp_bin);
                                uint8_t ascii_out[512];
                                uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
                                USART1_SendData(ascii_out, ascii_len);
                                
                                USART1_ClearCommand();
                                break; 
                            }
                            // 如果会话已损坏，忽略0x0503，等待上位机发0x0502重新开始
                        } else if (cmd_word == 0x0502) {
                            // 收到重新开始指令，复位状态，回到阶段1
                            session_invalid = FALSE;
                            fw_len = 0;
                            magic_checked = FALSE;
                            stream_end = FALSE;
                            
                            uint8_t ok = 0xFF;
                            uint8_t resp_bin[256];
                            uint16_t frame_len = comm_build_bin_frame(g_device_id, FRAME_TYPE_RESP, 0x0502, &ok, 1, resp_bin);
                            uint8_t ascii_out[512];
                            uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
                            USART1_SendData(ascii_out, ascii_len);
													delay_1ms(50);
													USART1_ClearCommand();
                        }
                    }
                }
            }
            USART1_ClearCommand();
        } else {
            // 超时检测：如果超过 1000ms 没有收到新数据，且已经收到了部分数据，认为裸流发送完毕
            if (!stream_end && fw_len > 0 && (sys_tick - last_recv_tick > 1000)) {
                stream_end = TRUE; // 切换到阶段2
            }
        }
    }

    if (fw_len == 0 || session_invalid) {
        return FALSE;
    }

    return perform_update(fw_buf, fw_len);
}

/**
 * @brief  将接收到的固件写入 App 区并校验 CRC
 */
bool perform_update(uint8_t *data, uint32_t size)
{
    // ??? 核心修改：如果存在魔术字头，将其剥离 ???
    uint32_t write_offset = 0;
    uint32_t write_size = size;

    if (size >= 4 && data[0] == 0x5A && data[1] == 0xA5 &&
        data[2] == 0xC3 && data[3] == 0x3C) {

        write_offset = 4;       // 跳过前4字节
        write_size -= 4;        // 写入总长度减4
    }

    if (write_size == 0 || write_size > 128 * 1024) return FALSE;

    uint32_t app_addr = my_param_sum.BootParam.appStartAddr;

    // 擦除 App 区（128KB）
    for (uint32_t i = 0; i < 128; i++) {
        internal_flash_erase(app_addr + i * 1024);
    }

    // 写入新固件 (从 data + write_offset 开始写，只写 write_size 长度)
    internal_flash_write_str_Char(app_addr, data + write_offset, write_size);

    // 读回验证 (也只验证剥掉头部后的真实数据)
    for (uint32_t i = 0; i < write_size; i++) {
        if (internal_flash_read_Char(app_addr + i) != data[write_offset + i]) {
            return FALSE;
        }
    }

    // 更新参数区中的 App 信息
    uint32_t new_crc = crc32_calc(data, size); // CRC依然对整个包计算，不影响
    my_param_sum.BootParam.appSize  = size;     // 记录原始包大小
    my_param_sum.BootParam.appCRC32 = new_crc;
    my_param_sum.BootParam.appVersion++;
    return TRUE;
}

/**
 * @brief  发送错误应答帧
 */
void send_error_response(uint16_t cmd_word, uint8_t err_code)
{
    cmd_word = 0xEEEE;
	uint8_t err = NULL;
    uint8_t resp_bin[256];
    uint16_t frame_len = comm_build_bin_frame(
        g_device_id, FRAME_TYPE_ERROR, cmd_word,
        &err, 0, resp_bin);
    uint8_t ascii_out[512];
    uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
    USART1_SendData(ascii_out, ascii_len);
}

/**
 * @brief  升级成功后更新参数区并写回 Flash (增加偏移兼容)
 */
void update_boot_param_after_upgrade(void)
{
    my_param_sum.BootParam.updateFlag   = 0x00;
    my_param_sum.BootParam.updateStatus = 0x00;
    my_param_sum.BootParam.updateCount++;
    my_param_sum.BootParam.appStartAddr = 0x08011000;
    my_param_sum.BootParam.appStackAddr = *(__IO uint32_t*)0x08011000;
    my_param_sum.BootParam.appEntryAddr = *(__IO uint32_t*)(0x08011000 + 4);

    my_param_sum.BootParam_Reserved.appVersion = my_param_sum.BootParam.appVersion;

    memcpy(config_buf, &my_param_sum, sizeof(Parameter_t));
    internal_flash_erase(BOOT_CONFIG_ADDR);
    internal_flash_write_str_Char(BOOT_CONFIG_ADDR, config_buf, CONFIG_SIZE);


}

/**
 * @brief  固件魔术字校验（弱定义）
 */
__weak int check_firmware_magic(void)
{
    if (fw_len < 4) return 0;
    if (fw_buf[0] == 0x5A && fw_buf[1] == 0xA5 &&
        fw_buf[2] == 0xC3 && fw_buf[3] == 0x3C)
        return 1;
    return 0;
}
/**
 * @brief  发送固件魔术字错误应答帧 (帧类型FF, 命令字EEEE)
 */
static void send_magic_error_frame(void)
{
    uint8_t resp_bin[256];
    // 0xFF 是错误帧类型，0xEEEE 是错误命令字，无内容体
    uint16_t frame_len = comm_build_bin_frame(
        g_device_id,
        0xFF,       // 帧类型 FF
        0xEEEE,     // 命令字 EEEE
        NULL,       // 无内容
        0,          // 内容长度 0
        resp_bin
    );
    uint8_t ascii_out[512];
    uint16_t ascii_len = comm_bin_to_ascii(resp_bin, frame_len, ascii_out);
    USART1_SendData(ascii_out, ascii_len);
}


/****************************End*****************************/