#include "zhenjiexi.h"
#include <string.h>

/* ================= CRC-16-Modbus 查表 ================= */
/* 正确的 CRC-16-Modbus 查表 */
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

 // CRC16计算函数 
uint16_t comm_crc16_modbus(const uint8_t *data, uint16_t length) 
 { 
     uint16_t crc = 0xFFFF; // MODBUS CRC16的初始值为0xFFFF 
  
     while(length--) 
     { 
         crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xFF]; 
     } 
  
     return crc; 
 } 
/* ================= 辅助：ASCII ? 二进制转换 ================= */
static uint8_t hex_char_to_nibble(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0xFF;
}

/* 将十六进制ASCII字符串转成二进制字节流，成功返回0 */
static int ascii_to_bin(const uint8_t *ascii, uint16_t ascii_len, uint8_t *bin) {
    if (ascii_len % 2) return -1;
    for (uint16_t i = 0; i < ascii_len; i += 2) {
        uint8_t high = hex_char_to_nibble(ascii[i]);
        uint8_t low  = hex_char_to_nibble(ascii[i + 1]);
        if (high > 0x0F || low > 0x0F) return -1;
        *bin++ = (high << 4) | low;
    }
    return 0;
}

/* 将二进制字节流转成十六进制ASCII字符串（大写） */
static void bin_to_ascii_hex(const uint8_t *bin, uint16_t bin_len, uint8_t *ascii_out) {
    static const char hex[] = "0123456789ABCDEF";
    for (uint16_t i = 0; i < bin_len; i++) {
        *ascii_out++ = hex[(bin[i] >> 4) & 0x0F];
        *ascii_out++ = hex[bin[i] & 0x0F];
    }
}

/* ================= 帧解析 ================= */
/**
 * @brief  解析纯十六进制 ASCII 帧（无帧头帧尾）
 * @param  ascii_frame : 完整 ASCII 字符串，如 "A5B6FFFF..."
 * @param  ascii_len   : 字符串长度
 * @param  bin_frame_out : 输出二进制帧
 * @param  bin_len_out   : 输出二进制帧长度
 * @return 0: 成功  -1: 失败
 */
int comm_parse_ascii_frame(const uint8_t *ascii_frame, uint16_t ascii_len,
                           uint8_t *bin_frame_out, uint16_t *bin_len_out) {
    // 1. 长度必须是偶数
    if (ascii_len % 2) {
        // 错误码 1：长度奇数

        return -1;
    }
    if (ascii_len < 13 * 2) {
        // 错误码 2：长度太短

        return -1;
    }

    // 2. ASCII → 二进制
    uint8_t bin_buf[256];
    if (ascii_to_bin(ascii_frame, ascii_len, bin_buf) != 0) {
        // 错误码 3：ascii_to_bin 失败

        return -1;
    }
    uint16_t bin_len = ascii_len / 2;

    // 3. 起始标志
    if (bin_buf[0] != 0xA5 || bin_buf[1] != 0xB6) {
        // 错误码 4：起始标志错误

        return -1;
    }

    // 4. 结束标志
    if (bin_buf[bin_len - 2] != 0xB6 || bin_buf[bin_len - 1] != 0xA5) {
        // 错误码 5：结束标志错误

        return -1;
    }

    // 5. 报文长度校验
    uint8_t content_len = bin_buf[7];
    if (bin_len != 13 + content_len) {
        // 错误码 6：长度不匹配

        return -1;
    }

    // 6. CRC 校验
    uint16_t crc_calc = comm_crc16_modbus(bin_buf, 9 + content_len);
    uint16_t crc_recv = ((uint16_t)bin_buf[bin_len - 4] << 8) | bin_buf[bin_len - 3];
    if (crc_calc != crc_recv) {
        // 错误码 7：CRC 错误

        return -1;
    }

    memcpy(bin_frame_out, bin_buf, bin_len);
    *bin_len_out = bin_len;
    return 0;
}

/* ================= 构建二进制帧 ================= */
uint16_t comm_build_bin_frame(uint16_t dev_id, uint8_t frame_type,
                              uint16_t cmd_word,
                              const uint8_t *content, uint8_t content_len,
                              uint8_t *bin_frame_out) {
    uint16_t idx = 0;

    // 起始标志
    bin_frame_out[idx++] = 0xA5;
    bin_frame_out[idx++] = 0xB6;

    // 设备ID
    bin_frame_out[idx++] = (dev_id >> 8) & 0xFF;
    bin_frame_out[idx++] = dev_id & 0xFF;

    // 帧类型
    bin_frame_out[idx++] = frame_type;

    // 命令字
    bin_frame_out[idx++] = (cmd_word >> 8) & 0xFF;
    bin_frame_out[idx++] = cmd_word & 0xFF;

    // 报文长度
    bin_frame_out[idx++] = content_len;

    // 协议版本
    bin_frame_out[idx++] = PROTOCOL_VERSION;

    // 内容
    if (content_len > 0 && content != NULL) {
        memcpy(&bin_frame_out[idx], content, content_len);
        idx += content_len;
    }

    // 计算 CRC（范围：当前已写入的所有字节，即起始标志到内容）
    uint16_t crc = comm_crc16_modbus(bin_frame_out, idx);
    bin_frame_out[idx++] = (crc >> 8) & 0xFF;   // 高字节在前
    bin_frame_out[idx++] = crc & 0xFF;

    // 结束标志
    bin_frame_out[idx++] = 0xB6;
    bin_frame_out[idx++] = 0xA5;

    return idx;  // 总长度 = 13 + content_len
}

/* ================= 二进制帧转 ASCII ================= */
/**
 * @brief  将二进制帧转换为纯十六进制 ASCII 字符串（无帧头帧尾）
 * @param  bin_frame : 二进制帧
 * @param  bin_len   : 二进制帧长度
 * @param  ascii_out : 输出缓冲区（需 >= bin_len * 2）
 * @return ASCII 字符串长度
 */
uint16_t comm_bin_to_ascii(const uint8_t *bin_frame, uint16_t bin_len,
                           uint8_t *ascii_out) {
    bin_to_ascii_hex(bin_frame, bin_len, ascii_out);
    return bin_len * 2;
}
/* ================= END ================= */