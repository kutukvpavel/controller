/**
 * @file dac_modules.h
 * @author Kutukov Pavel
 * @brief DAC communication routines
 * @version 0.1a
 * @date 2022-03-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "user.h"

#define MY_DAC_MAX_MODULES 4 //Hardware-determined

namespace dac
{
    struct module_t
    {
        SPI_HandleTypeDef* hspi;
        uint8_t cs_mux_mask; //MUX address port and mask
        I2C_HandleTypeDef* hi2c;
        uint8_t addr;
        bool present;
        float prev_current;
        float current;
        float setpoint;
        float corrected_setpoint;
        float depolarization_setpoint;
        float cal_coeff;
        float cal_offset;
        float current_cal_offset;
        float r_shunt;
        bool is_depolarizing;
    };

    //Globals
    extern volatile uint8_t status;
    extern user::pin_t enable_pin;
    extern user::pin_t* cs_pin; //main CS pin, not the MUX address pins
    extern GPIO_TypeDef* cs_mux_port;
    extern module_t modules[];

    // Public methods
    void init(SPI_HandleTypeDef* spi_instance, user::pin_t* spi_cs_pin, I2C_HandleTypeDef* i2c_instance);
    void probe();
    void set_module(size_t i, float volts);
    void set_all(float volts);
    void start_depolarization();
    void stop_depolarization();
    void read_current();
    void correct_for_current();
    void set_depolarization(float volts);
    size_t dump_module_report(char* buf, size_t max_len);
    size_t dump_last_data(char* buf, size_t max_len);
}