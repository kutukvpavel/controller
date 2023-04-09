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
    HAL_StatusTypeDef reset();
    void dump_hex();
    HAL_StatusTypeDef test();

    PACKED_FOR_MODBUS motor_params_t* get_motor_params(size_t i);
    PACKED_FOR_MODBUS a_io::in_cal_t* get_analog_input_cal(size_t i);
    PACKED_FOR_MODBUS a_io::in_cal_t* get_temp_sensor_cal();
    PACKED_FOR_MODBUS adc::ch_cal_t* get_adc_channel_cal(size_t i);
    PACKED_FOR_MODBUS dac::cal_t* get_dac_cal(size_t i);
} // namespace nvs
