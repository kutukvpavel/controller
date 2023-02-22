/**
 * @file commands.h
 * @author Pavel Kutukov
 * @brief FOR TESTING until modbus is implemented
 * @version 0.1
 * @date 2022-03-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "user.h"
#include "motor.h"

#include <inttypes.h>
#include <stddef.h>

#define MY_CMD_ACQUIRE _BV(0)
#define MY_CMD_STATUS_DAC_CORRECT _BV(1) //Single-shot
#define MY_CMD_STATUS_MOTOR_EN _BV(2)
#define MY_CMD_STATUS_READY _BV(3)
#define MY_CMD_STATUS_SAVE_EEPROM _BV(4)
#define MY_CMD_STATUS_HAVE_NEW_DATA _BV(5)

#define DEPOLARIZATION_REPORT_FORMAT "BFF: %3.2f\n"

namespace cmd
{
    typedef uint8_t bitfield_t;

    void init(user::Stream&, I2C_HandleTypeDef*);
    void poll();

    void report_ready();

    void set_status_bit(bitfield_t mask);
    void reset_status_bit(bitfield_t mask);
    bool get_status_bit_set(bitfield_t bitmask);
    float get_dac_setpoint(size_t i);
    void set_adc_voltage(size_t i, float v);

    motor_params_t* get_motor_params(size_t i);
    a_io::in_cal_t* get_analog_input_cal(size_t i);
    a_io::in_cal_t* get_temp_sensor_cal();
    adc::ch_cal_t* get_adc_channel_cal(size_t i);
    dac::cal_t* get_dac_cal(size_t i);
}