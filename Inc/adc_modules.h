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

//PRIVATE

//PUBLIC

namespace adc
{
    struct channel_t
    {
        int mux_conf;
        float cal_coeff;
        float cal_offset;
        float last_result;
        bool invert;
    };
    struct module_t
    {
        SPI_HandleTypeDef* hspi;
        uint8_t cs_mux_mask;
        user::pin_t* drdy;
        bool present;
        uint8_t selected_channel;
        uint8_t last_channel;
        channel_t channels[MY_ADC_CHANNELS_PER_CHIP];
    };

    //Globals
    extern volatile uint8_t status;
    extern int16_t acquisition_speed;
    extern user::pin_t drdy_pin;
    extern user::pin_t enable_pin;
    extern user::pin_t* cs_pin; //main CS pin, not the MUX address pins
    extern GPIO_TypeDef* cs_mux_port;
    extern module_t modules[];

    //Public methods
    void init(SPI_HandleTypeDef* hspi, user::pin_t* spi_cs_pin);
    void probe();
    void increment_and_sync();
    void read();
    void drdy_callback();
    size_t dump_last_data(char* buf, size_t max_len);
    void drdy_check();
    size_t dump_module_report(char* buf, size_t max_len);
}

