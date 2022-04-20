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

#define MY_CMD_ACQUIRE _BV(0)
#define MY_CMD_STATUS_DAC_CORRECT _BV(1) //Single-shot

#define DEPOLARIZATION_REPORT_FORMAT "BFF: %3.2f\n"

namespace cmd
{
    extern uint8_t status;
    extern float dac_setpoint;
    extern float depolarization_percent;
    extern float depolarization_setpoint;

    void report_ready();
    void process(user::Stream* stream, char* output_buf, size_t max_len);
    size_t report_depolarization_percent(char* output_buf, size_t max_len);
}