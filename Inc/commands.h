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

namespace cmd
{
    void report_ready();
    void process(user::Stream* stream);
}