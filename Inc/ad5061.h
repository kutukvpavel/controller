/**
 * @file ad5061.h
 * @author Kutukov Pavel
 * @brief AD5061 DAC driver for STM32 HAL
 * @version 0.1
 * @date 2022-03-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "main.h"

#ifndef _BV
	#define _BV(val) (1u << val)
#endif

#define AD5061_MODE_NORMAL 0x00
#define AD5061_MODE_TRISTATE _BV(16)
#define AD5061_MODE_100K_GND _BV(17)
#define AD5061_MODE_1K_GND (_BV(16) | _BV(17))

void ad5061_set_code(SPI_HandleTypeDef* hspi, uint16_t code);
void ad5061_set_mode(SPI_HandleTypeDef* hspi, uint32_t mode);
void ad5061_reset(SPI_HandleTypeDef* hspi);