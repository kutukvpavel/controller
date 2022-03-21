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
#define MY_CMD_DAC_SETPOINT_CHANGED _BV(1)
#define MY_CMD_DAC_DEPOLARIZATION_CHANGED _BV(2)
#define MY_CMD_STATUS_DAC_CORRECTION _BV(3)

namespace cmd
{
    extern uint8_t status;
    extern float dac_setpoint;
    extern uint16_t depolarization_time;
    extern float depolarization_setpoint;

    void report_ready();
    void process(user::Stream* stream);
}