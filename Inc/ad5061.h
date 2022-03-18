/**
 * @file ad5061.h
 * @author Kutukov Pavel
 * @brief AD5061 DAC driver for STM32 HAL
 * @version 0.1
 * @date 2022-03-12
 * 
 * @copyright Copyright (c) 2022
 * 
 * WARNING: this DAC requires /CS to return to HIGH before every write cycle,
 * therefore STM32F4 Hardware NSS pin mode won't work (it stays low as long as SPI1 is enabled)!
 * 
 */

#pragma once

#include "main.h"

#ifndef _BV
	#define _BV(val) (1u << (val))
#endif

#define AD5061_MODE_NORMAL 0x00
#define AD5061_MODE_TRISTATE _BV(16)
#define AD5061_MODE_100K_GND _BV(17)
#define AD5061_MODE_1K_GND (_BV(16) | _BV(17))
#define AD5061_FULL_SCALE 0xFFFF

#define AD5061_REFERENCE_VOLTAGE 2.5 //V
#define AD5061_INTERNAL_RESISTANCE 0.015

void ad5061_set_code(SPI_HandleTypeDef* hspi, uint16_t code);
void ad5061_set_mode(SPI_HandleTypeDef* hspi, uint32_t mode);
void ad5061_reset(SPI_HandleTypeDef* hspi);