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
    typedef uint8_t status_t;

    void init(user::Stream&, const motor_params_t*);
    void poll();

    void report_ready();

    void set_status_bit(status_t mask);
    void reset_status_bit(status_t mask);
    bool get_status_bit_set(status_t bitmask);
    const motor_params_t* get_motor_params(size_t i);
    float get_dac_setpoint();
    float get_depolarization_percent();
    float get_depolarization_setpoint();

    //void process(user::Stream* stream, char* output_buf, size_t max_len);
    //size_t report_depolarization_percent(char* output_buf, size_t max_len);
}