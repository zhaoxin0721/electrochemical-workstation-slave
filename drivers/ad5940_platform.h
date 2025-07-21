//此项目为新建文件
#pragma once

// 选择平台
#define USE_NRF52832

#ifdef USE_NRF52832
#include "ad5940_nrf52.h"
#else
// 这里可以放原有的ad5940平台相关头文件
#endif

// 平台相关函数声明（无论哪个平台都需要声明）
void      AD5940_CsClr(void);
void      AD5940_CsSet(void);
void      AD5940_RstClr(void);
void      AD5940_RstSet(void);
void      AD5940_Delay10us(uint32_t time);
void      AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuff, unsigned long length);
