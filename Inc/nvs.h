#pragma once

#include "motor.h"
#include "a_io.h"
#include "adc_modules.h"
#include "dac_modules.h"

namespace nvs
{
    HAL_StatusTypeDef init(I2C_HandleTypeDef*);
    HAL_StatusTypeDef save();
    HAL_StatusTypeDef load();

    motor_params_t* get_motor_params(size_t i);
    a_io::in_cal_t* get_analog_input_cal(size_t i);
    a_io::in_cal_t* get_temp_sensor_cal();
    adc::ch_cal_t* get_adc_channel_cal(size_t i);
    dac::cal_t* get_dac_cal(size_t i);
} // namespace nvs
