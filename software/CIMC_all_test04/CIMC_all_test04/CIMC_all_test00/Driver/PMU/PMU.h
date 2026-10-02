/************************************************************
 * 版权：2025CIMC Copyright。
 * 文件：PMU.h
 * 作者: Jialei Zhao
 * 平台: 2025CIMC IHD-V04
 * 版本: Jialei Zhao     2026/06/01     V0.01    original
************************************************************/

#ifndef __PMU_H
#define __PMU_H

#include "HeaderFiles.h"

void lowpower_sleep(void);
void lowpower_deepsleep(void);
void lowpower_standby(void);
static void rcu_config(void);
void lowpower_init(void);
#endif

