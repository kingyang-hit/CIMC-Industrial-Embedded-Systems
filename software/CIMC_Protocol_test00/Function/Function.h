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


/************************ 函数定义 ************************/

void System_Init(void);      	// 系统初始化
void UsrFunction(void);         // 用户函数
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

#endif


/****************************End*****************************/

