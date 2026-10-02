#ifndef ZHENJIEXI_H
#define ZHENJIEXI_H

#include <stdint.h>

/* ========== 帧格式常量 ========== */
#define FRAME_START_VALUE   0xA5B6
#define FRAME_END_VALUE     0xB6A5
#define PROTOCOL_VERSION    0x02

#define FRAME_TYPE_CMD      0x01
#define FRAME_TYPE_RESP     0x02
#define FRAME_TYPE_HEART    0x05
#define FRAME_TYPE_ERROR    0xFF

/* ========== API ========== */

/**
 * @brief  解析上位机发来的 ASCII 帧（如 ":A5B6...\r\n"），
 *         校验起始/结束标志、长度、CRC，提取出二进制帧。
 * @param  ascii_frame    : 完整的 ASCII 字符串缓冲区
 * @param  ascii_len      : 字符串长度
 * @param  bin_frame_out  : 输出二进制帧的缓冲区
 * @param  bin_len_out    : 输出二进制帧的实际长度（字节数）
 * @return 0: 解析成功  -1: 格式错误 / 校验失败
 */
int comm_parse_ascii_frame(const uint8_t *ascii_frame, uint16_t ascii_len,
                           uint8_t *bin_frame_out, uint16_t *bin_len_out);

/**
 * @brief  构建一帧完整的二进制帧（含起始/结束标志、CRC，不含 ASCII 封装）。
 *         参数均为逻辑层要发送的字段。
 * @param  dev_id         设备地址（16位大端，本机ID）
 * @param  frame_type     帧类型
 * @param  cmd_word       命令字
 * @param  content        内容数据指针
 * @param  content_len    内容长度（字节数）
 * @param  bin_frame_out  输出二进制帧缓冲区（需保证 >= 13 + content_len）
 * @return 二进制帧的总长度（字节数）
 */
uint16_t comm_build_bin_frame(uint16_t dev_id, uint8_t frame_type,
                              uint16_t cmd_word,
                              const uint8_t *content, uint8_t content_len,
                              uint8_t *bin_frame_out);

/**
 * @brief  将二进制帧转换为带帧头帧尾的 ASCII 字符串。
 *         输出格式：':' + 十六进制ASCII + "\r\n"
 * @param  bin_frame  二进制帧数据
 * @param  bin_len    二进制帧长度
 * @param  ascii_out  输出 ASCII 字符串缓冲区（需 >= 2 + bin_len*2 + 2）
 * @return ASCII 字符串的长度（不含 '\0'）
 */
uint16_t comm_bin_to_ascii(const uint8_t *bin_frame, uint16_t bin_len,
                           uint8_t *ascii_out);

/**
 * @brief  CRC-16-Modbus 计算（提供给外部校验或调试用）
 */
uint16_t comm_crc16_modbus(const uint8_t *data, uint16_t len);

#endif /* ZHENJIEXI_H */
