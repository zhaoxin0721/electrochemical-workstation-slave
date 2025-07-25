#ifndef DAC_H
#define DAC_H                           

#include <stdint.h>

int dac_init(void);
int dac_write(uint16_t value);

#endif
/******************************************************************************
 * @file dac.c
 * @brief DAC driver for MCP4921 DAC chip.
 * 
 * This file contains functions to initialize the DAC and write values to it.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 Your Company. All rights reserved.</center></h2>
 *
 * This software component is licensed by Your Company under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
