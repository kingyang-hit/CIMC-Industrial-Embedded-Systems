#include "SPI_FLASH.h"

#define WRITE            0x02     /* write to memory instruction */
#define WRSR             0x01     /* write status register instruction */
#define WREN             0x06     /* write enable instruction */

#define READ             0x03     /* read from memory instruction */
#define RDSR             0x05     /* read status register instruction  */
#define RDID             0x9F     /* read identification */
#define SE               0x20     /* sector erase instruction */
#define BE               0xC7     /* bulk erase instruction */

#define WIP_FLAG         0x01     /* write in progress(wip)flag */
#define DUMMY_BYTE       0xA5
// ========== 波特率映射表 ==========
static const Baudrate_Map_t baudrate_map[BAUDRATE_MAP_SIZE] = {
    {17, 4800},
    {18, 9600},
    {19, 19200},
    {20, 115200}
};
//26
extern uint16_t g_device_id;
// ========== 全局变量定义 ==========
uint8_t  g_baudrate_index = DEFAULT_BAUDRATE_INDEX;
uint32_t current_baudrate = DEFAULT_BAUDRATE_VALUE;
uint8_t  baudrate_index = DEFAULT_BAUDRATE_INDEX;  // 新增：波特率索引

float    ch0_ratio = DEFAULT_CH0_RATIO;
float    ch1_ratio = DEFAULT_CH1_RATIO;
float    ch0_thresh = DEFAULT_CH0_THRESH;
float    ch1_thresh = DEFAULT_CH1_THRESH;
uint8_t alarm_report_mode = DEFAULT_ALARM_MODE;


/*!
    \brief      initialize SPI1 GPIO and parameter
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_flash_init(void)
{
    spi_parameter_struct spi_init_struct;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);

	
	 /* SPI1_CLK(PB13), SPI1_MISO(PB14), SPI1_MOSI(PB15) */
    gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_13|GPIO_PIN_14| GPIO_PIN_15);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13|GPIO_PIN_14| GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_13|GPIO_PIN_14| GPIO_PIN_15);

    /* SPI1_CS(PB12) GPIO pin configuration */
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);


    /* chip select invalid*/
    SPI_FLASH_CS_HIGH();

    /* SPI1 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8 ;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;;
    spi_init(SPI1, &spi_init_struct);

    /* enable SPI1 */
    spi_enable(SPI1);
		
		
	 uint32_t temp = read_uint32_with_default(FLASH_ADDR_DEVICE_ID, DEFAULT_DEVICE_ID);
    g_device_id = (uint16_t)temp;
     flash_load_baudrate();
		
    ch0_ratio = read_float_with_default(FLASH_ADDR_CH0_RATIO, DEFAULT_CH0_RATIO);
    ch1_ratio = read_float_with_default(FLASH_ADDR_CH1_RATIO, DEFAULT_CH1_RATIO);
    ch0_thresh = read_float_with_default(FLASH_ADDR_CH0_THRESH, DEFAULT_CH0_THRESH);
    ch1_thresh = read_float_with_default(FLASH_ADDR_CH1_THRESH, DEFAULT_CH1_THRESH);
		alarm_report_mode = read_uint8_with_default(FLASH_ADDR_ALARM_MODE, DEFAULT_ALARM_MODE);
		 extern uint8_t g_alarm_enable;
    g_alarm_enable = alarm_report_mode;
     extern float g_ch0_ratio, g_ch1_ratio, g_yuzhi_ch0, g_yuzhi_ch1;
    g_ch0_ratio = ch0_ratio;
    g_ch1_ratio = ch1_ratio;
    g_yuzhi_ch0 = ch0_thresh;
    g_yuzhi_ch1 = ch1_thresh;
}

/*!
    \brief      erase the specified flash sector
    \param[in]  sector_addr: address of the sector to erase
    \param[out] none
    \retval     none
*/
void spi_flash_sector_erase(uint32_t sector_addr)
{
    /* send write enable instruction */
    spi_flash_write_enable();

    /* sector erase */
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send sector erase instruction */
    spi_flash_send_byte(SE);
    /* send sector_addr high nibble address byte */
    spi_flash_send_byte((sector_addr & 0xFF0000) >> 16);
    /* send sector_addr medium nibble address byte */
    spi_flash_send_byte((sector_addr & 0xFF00) >> 8);
    /* send sector_addr low nibble address byte */
    spi_flash_send_byte(sector_addr & 0xFF);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();

    /* wait the end of flash writing */
    spi_flash_wait_for_write_end();
}

/*!
    \brief      erase the entire flash
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_flash_bulk_erase(void)
{
    /* send write enable instruction */
    spi_flash_write_enable();

    /* bulk erase */
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();
    /* send bulk erase instruction  */
    spi_flash_send_byte(BE);
    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();

    /* wait the end of flash writing */
    spi_flash_wait_for_write_end();
}

/*!
    \brief      write more than one byte to the flash
    \param[in]  pbuffer: pointer to the buffer
    \param[in]  write_addr: flash's internal address to write
    \param[in]  num_byte_to_write: number of bytes to write to the flash
    \param[out] none
    \retval     none
*/
void spi_flash_page_write(uint8_t* pbuffer, uint32_t write_addr, uint16_t num_byte_to_write)
{
    /* enable the write access to the flash */
    spi_flash_write_enable();

    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "write to memory" instruction */
    spi_flash_send_byte(WRITE);
    /* send write_addr high nibble address byte to write to */
    spi_flash_send_byte((write_addr & 0xFF0000) >> 16);
    /* send write_addr medium nibble address byte to write to */
    spi_flash_send_byte((write_addr & 0xFF00) >> 8);
    /* send write_addr low nibble address byte to write to */
    spi_flash_send_byte(write_addr & 0xFF);

    /* while there is data to be written on the flash */
    while(num_byte_to_write--){
        /* send the current byte */
        spi_flash_send_byte(*pbuffer);
        /* point on the next byte to be written */
        pbuffer++;
    }

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();

    /* wait the end of flash writing */
    spi_flash_wait_for_write_end();
}

/*!
    \brief      write block of data to the flash
    \param[in]  pbuffer: pointer to the buffer
    \param[in]  write_addr: flash's internal address to write
    \param[in]  num_byte_to_write: number of bytes to write to the flash
    \param[out] none
    \retval     none
*/
void spi_flash_buffer_write(uint8_t* pbuffer, uint32_t write_addr, uint16_t num_byte_to_write)
{
    uint8_t num_of_page = 0, num_of_single = 0, addr = 0, count = 0, temp = 0;

    addr          = write_addr % SPI_FLASH_PAGE_SIZE;
    count         = SPI_FLASH_PAGE_SIZE - addr;
    num_of_page   = num_byte_to_write / SPI_FLASH_PAGE_SIZE;
    num_of_single = num_byte_to_write % SPI_FLASH_PAGE_SIZE;

     /* write_addr is SPI_FLASH_PAGE_SIZE aligned  */
    if(0 == addr){
        /* num_byte_to_write < SPI_FLASH_PAGE_SIZE */
        if(0 == num_of_page)
            spi_flash_page_write(pbuffer,write_addr,num_byte_to_write);
        /* num_byte_to_write > SPI_FLASH_PAGE_SIZE */
        else{
            while(num_of_page--){
                spi_flash_page_write(pbuffer,write_addr,SPI_FLASH_PAGE_SIZE);
                write_addr += SPI_FLASH_PAGE_SIZE;
                pbuffer += SPI_FLASH_PAGE_SIZE;
            }
            spi_flash_page_write(pbuffer,write_addr,num_of_single);
        }
    }else{
        /* write_addr is not SPI_FLASH_PAGE_SIZE aligned  */
        if(0 == num_of_page){
            /* (num_byte_to_write + write_addr) > SPI_FLASH_PAGE_SIZE */
            if(num_of_single > count){
                temp = num_of_single - count;
                spi_flash_page_write(pbuffer,write_addr,count);
                write_addr += count;
                pbuffer += count;
                spi_flash_page_write(pbuffer,write_addr,temp);
            }else
                spi_flash_page_write(pbuffer,write_addr,num_byte_to_write);
        }else{
            /* num_byte_to_write > SPI_FLASH_PAGE_SIZE */
            num_byte_to_write -= count;
            num_of_page = num_byte_to_write / SPI_FLASH_PAGE_SIZE;
            num_of_single = num_byte_to_write % SPI_FLASH_PAGE_SIZE;

            spi_flash_page_write(pbuffer,write_addr, count);
            write_addr += count;
            pbuffer += count;

            while(num_of_page--){
                spi_flash_page_write(pbuffer,write_addr,SPI_FLASH_PAGE_SIZE);
                write_addr += SPI_FLASH_PAGE_SIZE;
                pbuffer += SPI_FLASH_PAGE_SIZE;
            }

            if(0 != num_of_single)
                spi_flash_page_write(pbuffer,write_addr,num_of_single);
        }
    }
}

/*!
    \brief      read a block of data from the flash
    \param[in]  pbuffer: pointer to the buffer that receives the data read from the flash
    \param[in]  read_addr: flash's internal address to read from
    \param[in]  num_byte_to_read: number of bytes to read from the flash
    \param[out] none
    \retval     none
*/
void spi_flash_buffer_read(uint8_t* pbuffer, uint32_t read_addr, uint16_t num_byte_to_read)
{
    /* select the flash: chip slect low */
    SPI_FLASH_CS_LOW();

    /* send "read from memory " instruction */
    spi_flash_send_byte(READ);

    /* send read_addr high nibble address byte to read from */
    spi_flash_send_byte((read_addr & 0xFF0000) >> 16);
    /* send read_addr medium nibble address byte to read from */
    spi_flash_send_byte((read_addr& 0xFF00) >> 8);
    /* send read_addr low nibble address byte to read from */
    spi_flash_send_byte(read_addr & 0xFF);

    /* while there is data to be read */
    while(num_byte_to_read--){
        /* read a byte from the flash */
        *pbuffer = spi_flash_send_byte(DUMMY_BYTE);
        /* point to the next location where the byte read will be saved */
        pbuffer++;
    }

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

/*!
    \brief      read flash identification
    \param[in]  none
    \param[out] none
    \retval     flash identification
*/
uint32_t spi_flash_read_id(void)
{
    uint32_t temp = 0, temp0 = 0, temp1 = 0, temp2 = 0;

    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "RDID " instruction */
    spi_flash_send_byte(0x9F);

    /* read a byte from the flash */
    temp0 = spi_flash_send_byte(DUMMY_BYTE);

    /* read a byte from the flash */
    temp1 = spi_flash_send_byte(DUMMY_BYTE);

    /* read a byte from the flash */
    temp2 = spi_flash_send_byte(DUMMY_BYTE);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();

    temp = (temp0 << 16) | (temp1 << 8) | temp2;

    return temp;
}

/*!
    \brief      initiate a read data byte (read) sequence from the flash
    \param[in]  read_addr: flash's internal address to read from
    \param[out] none
    \retval     none
*/
void spi_flash_start_read_sequence(uint32_t read_addr)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "read from memory " instruction */
    spi_flash_send_byte(READ);

    /* send the 24-bit address of the address to read from */
    /* send read_addr high nibble address byte */
    spi_flash_send_byte((read_addr & 0xFF0000) >> 16);
    /* send read_addr medium nibble address byte */
    spi_flash_send_byte((read_addr& 0xFF00) >> 8);
    /* send read_addr low nibble address byte */
    spi_flash_send_byte(read_addr & 0xFF);
}

/*!
    \brief      read a byte from the SPI flash
    \param[in]  none
    \param[out] none
    \retval     byte read from the SPI flash
*/
uint8_t spi_flash_read_byte(void)
{
    return(spi_flash_send_byte(DUMMY_BYTE));
}

/*!
    \brief      send a byte through the SPI interface and return the byte received from the SPI bus
    \param[in]  byte: byte to send
    \param[out] none
    \retval     the value of the received byte
*/
uint8_t spi_flash_send_byte(uint8_t byte)
{
    /* loop while data register in not emplty */
    while (RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_TBE));

    /* send byte through the SPI1 peripheral */
    spi_i2s_data_transmit(SPI1,byte);

    /* wait to receive a byte */
    while(RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_RBNE));

    /* return the byte read from the SPI bus */
    return(spi_i2s_data_receive(SPI1));
}

/*!
    \brief      send a half word through the SPI interface and return the half word received from the SPI bus
    \param[in]  half_word: half word to send
    \param[out] none
    \retval     the value of the received byte
*/
uint16_t spi_flash_send_halfword(uint16_t half_word)
{
    /* loop while data register in not emplty */
    while(RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_TBE));

    /* send half word through the SPI1 peripheral */
    spi_i2s_data_transmit(SPI1,half_word);

    /* wait to receive a half word */
    while(RESET == spi_i2s_flag_get(SPI1,SPI_FLAG_RBNE));

    /* return the half word read from the SPI bus */
    return spi_i2s_data_receive(SPI1);
}

/*!
    \brief      enable the write access to the flash
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_flash_write_enable(void)
{
    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "write enable" instruction */
    spi_flash_send_byte(WREN);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

/*!
    \brief      poll the status of the write in progress(wip) flag in the flash's status register
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_flash_wait_for_write_end(void)
{
    uint8_t flash_status = 0;

    /* select the flash: chip select low */
    SPI_FLASH_CS_LOW();

    /* send "read status register" instruction */
    spi_flash_send_byte(RDSR);

    /* loop as long as the memory is busy with a write cycle */
    do{
        /* send a dummy byte to generate the clock needed by the flash
        and put the value of the status register in flash_status variable */
        flash_status = spi_flash_send_byte(DUMMY_BYTE);
    }while((flash_status & WIP_FLAG) == SET);

    /* deselect the flash: chip select high */
    SPI_FLASH_CS_HIGH();
}

// ========== 扇区擦除 + 写入（封装已有的函数） ==========
void flash_sector_write(uint32_t addr, uint8_t *data, uint32_t len)
{
   uint32_t sector_addr = (addr / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    
    // 检查是否需要擦除
    uint8_t check_byte;
    spi_flash_buffer_read(&check_byte, sector_addr, 1);
    
    if (check_byte != 0xFF) {
        spi_flash_sector_erase(sector_addr);
    }
    
    spi_flash_buffer_write(data, addr, len);
}

// ========== 读取并判断有效性 ==========
 uint32_t read_uint32_with_default(uint32_t addr, uint32_t default_val)
{
    uint32_t val;
    spi_flash_buffer_read((uint8_t*)&val, addr, 4);
    return (val == 0xFFFFFFFF) ? default_val : val;
}

 uint8_t read_uint8_with_default(uint32_t addr, uint8_t default_val)
{
    uint8_t val;
    spi_flash_buffer_read(&val, addr, 1);
    return (val == 0xFF) ? default_val : val;
}

float read_float_with_default(uint32_t addr, float default_val)
{
    uint32_t val;
    spi_flash_buffer_read((uint8_t*)&val, addr, 4);
    if (val == 0xFFFFFFFF) return default_val;
    return *(float*)&val;
}
// ========== 保存函数（每行只有一行调用） ==========
void flash_save_device_id(void)
{
     uint32_t temp = g_device_id;
    flash_sector_write(FLASH_ADDR_DEVICE_ID, (uint8_t*)&temp, 4);
}

void flash_save_baudrate(void)
{
   flash_save_baudrate_with_index(g_baudrate_index);
}

void flash_save_ch0_ratio(void)
{
    flash_sector_write(FLASH_ADDR_CH0_RATIO, (uint8_t*)&ch0_ratio, 4);
}

void flash_save_ch1_ratio(void)
{
    flash_sector_write(FLASH_ADDR_CH1_RATIO, (uint8_t*)&ch1_ratio, 4);
}

void flash_save_ch0_thresh(void)
{
    flash_sector_write(FLASH_ADDR_CH0_THRESH, (uint8_t*)&ch0_thresh, 4);
}

void flash_save_ch1_thresh(void)
{
    flash_sector_write(FLASH_ADDR_CH1_THRESH, (uint8_t*)&ch1_thresh, 4);
}
void flash_save_alarm_mode(void) {
   
    flash_sector_write(FLASH_ADDR_ALARM_MODE, &alarm_report_mode, 1);
}
// 新增：从Flash读取波特率配置
// 新增：从Flash读取波特率配置
void read_baudrate_from_flash(void)
{
    uint8_t baud_data[5];  // 1字节索引 + 4字节值
    Baudrate_Config_t default_config = {
        .index = DEFAULT_BAUDRATE_INDEX,
        .value = DEFAULT_BAUDRATE_VALUE
    };
    
    // 读取5字节数据
    spi_flash_buffer_read(baud_data, FLASH_ADDR_BAUDRATE, 5);
    
    // 检查是否是有效数据（第一个字节不是0xFF）
    if (baud_data[0] != 0xFF) {
        baudrate_index = baud_data[0];
        current_baudrate = ((uint32_t)baud_data[1] << 24) |
                          ((uint32_t)baud_data[2] << 16) |
                          ((uint32_t)baud_data[3] << 8) |
                          baud_data[4];
        
        // 验证读取的值是否合理
        if (current_baudrate == 0xFFFFFFFF || current_baudrate == 0) {
            baudrate_index = default_config.index;
            current_baudrate = default_config.value;
        }
    } else {
        // 使用默认值
        baudrate_index = default_config.index;
        current_baudrate = default_config.value;
    }
}


// 新增：单独保存波特率索引
void flash_save_baudrate_index(uint8_t index)
{
    baudrate_index = index;
    flash_save_baudrate();  // 调用完整保存函数
}

// 新增：单独保存波特率值
void flash_save_baudrate_value(uint32_t value)
{
    current_baudrate = value;
    flash_save_baudrate();  // 调用完整保存函数
}
/**
 * @brief 根据索引获取波特率值
 * @param index: 索引(17-20)
 * @param out_value: 输出波特率值
 * @return 0=成功, 1=无效索引
 */
uint8_t baudrate_index_to_value(uint8_t index, uint32_t *out_value) {
    for (uint8_t i = 0; i < BAUDRATE_MAP_SIZE; i++) {
        if (baudrate_map[i].index == index) {
            *out_value = baudrate_map[i].baudrate;
            return 0;  // 成功
        }
    }
    return 1;  // 无效索引
}
/**
 * @brief 根据波特率值获取索引
 * @param value: 波特率值
 * @return 索引(17-20)，未找到返回默认值
 */
uint8_t baudrate_value_to_index(uint32_t value) {
    for (uint8_t i = 0; i < BAUDRATE_MAP_SIZE; i++) {
        if (baudrate_map[i].baudrate == value) {
            return baudrate_map[i].index;
        }
    }
    return DEFAULT_BAUDRATE_INDEX;
}

/**
 * @brief 保存波特率（带索引）到Flash
 * @param index: 波特率索引
 */
void flash_save_baudrate_with_index(uint8_t index) {
    uint32_t baudrate = 0;
    
    // 获取对应的波特率值
    if (baudrate_index_to_value(index, &baudrate) != 0) {
        return;  // 无效索引，不保存
    }
    
    // 更新全局变量
    g_baudrate_index = index;
    current_baudrate = baudrate;
    
    // 构建存储结构
    Baudrate_Config_t config;
    config.index = index;
    config.value = baudrate;
    
    // 擦除扇区并写入
    spi_flash_sector_erase(FLASH_ADDR_BAUDRATE);
    spi_flash_buffer_write((uint8_t*)&config, FLASH_ADDR_BAUDRATE, sizeof(Baudrate_Config_t));
}
/**
 * @brief 从Flash加载波特率
 */
void flash_load_baudrate(void) {
    Baudrate_Config_t config;
    
    // 从Flash读取
    spi_flash_buffer_read((uint8_t*)&config, FLASH_ADDR_BAUDRATE, sizeof(Baudrate_Config_t));
    
    // 检查是否有效（index应该在17-20范围内）
    if (config.index >= 17 && config.index <= 20) {
        // 验证索引和值是否匹配
        uint32_t expected_value = 0;
        if (baudrate_index_to_value(config.index, &expected_value) == 0) {
            if (config.value == expected_value) {
                // 数据有效，使用Flash中的值
                g_baudrate_index = config.index;
                current_baudrate = config.value;
                return;
            }
        }
    }
    
    // Flash数据无效，使用默认值
    g_baudrate_index = DEFAULT_BAUDRATE_INDEX;
    current_baudrate = DEFAULT_BAUDRATE_VALUE;
}
/**
 * @brief 上电初始化，读取所有参数
 */
void flash_params_init(void) {
    // 加载波特率
    flash_load_baudrate();
    
    // 加载设备ID
    g_device_id = read_uint32_with_default(FLASH_ADDR_DEVICE_ID, DEFAULT_DEVICE_ID);
    
    // 加载变比
    ch0_ratio = read_float_with_default(FLASH_ADDR_CH0_RATIO, DEFAULT_CH0_RATIO);
    ch1_ratio = read_float_with_default(FLASH_ADDR_CH1_RATIO, DEFAULT_CH1_RATIO);
    
    // 加载阈值
    ch0_thresh = read_float_with_default(FLASH_ADDR_CH0_THRESH, DEFAULT_CH0_THRESH);
    ch1_thresh = read_float_with_default(FLASH_ADDR_CH1_THRESH, DEFAULT_CH1_THRESH);
    
    // 加载告警模式
    alarm_report_mode = read_uint8_with_default(FLASH_ADDR_ALARM_MODE, DEFAULT_ALARM_MODE);
}

