/**
 * @file adc_modules.h
 * @author Kutukov Pavel
 * @brief ADC acquisition logic
 * @version 0.1a
 * @date 2022-01-28
 * 
 * @copyright Copyright (c) 2022
 * 
 * STATE MACHINE
 * 0. Initialize configuration registers (free-running mode + DRDYM bit),
 *      send sync commands as close to each other as possible
 *   ↓
 * 1. Conversion in progress, do other things      ← ← ← ← ←
 * 2. /DRDY transitions low (interrupt?)                    ↑
 * 3. Acquire data (deferred exec?):                        ↑
 *    Select i-th module                ← ← ← ← ← ← ← ← ←   ↑
 *    Wait for it's DOUT/nDRDY to transition to LOW     ↑   ↑
 *    Clock the data out                                ↑   ↑
 *      Repreat for all modules in SYNC order → → → → → → → ↑
 *
 *  I.e.: init → conv pending → read pending → reading → ↓
 *                         ↑ ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←
 */

#pragma once

#include "user.h"

#define MY_ADC_MAX_MODULES 4 //Hardware-determined
#define MY_ADC_CHANNELS_PER_CHIP 2

#define MY_ADC_STATUS_INITIALIZING 0
#define MY_ADC_STATUS_WAITING 1
#define MY_ADC_STATUS_READ_PENDING 2
#define MY_ADC_STATUS_READING 3

namespace adc
{
    //Globals
    extern volatile uint8_t status;
    extern float calibration_coefficients[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP];
    extern float calibration_offset[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP];
    extern int16_t channel_gain[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP];
    extern int16_t acquisition_speed;
    extern uint16_t acquisition_period;
    extern bool module_present[MY_ADC_MAX_MODULES];

    //Public methods
    void probe();
    void initialize();
    void read();
}

