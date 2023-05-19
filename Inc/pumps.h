#pragma once

#include "user.h"
#include "motor.h"
#include "sr_io.h"

namespace pumps
{
    struct PACKED_FOR_MODBUS params_t
    {
        float kP;
        float kI;
        float kD;
        uint16_t low_concentration_motor_index;
        uint16_t high_concentration_motor_index;
        uint16_t sensing_adc_channel_index;
        uint16_t low_concentration_dac_channel_index;
        uint16_t high_concentration_dac_channel_index;
        uint16_t low_conc_adc_ch_index;
        uint16_t high_conc_adc_ch_index;
        uint16_t reserved1; //Alignment
        float total_flowrate;
    };
    enum mode_t
    {
        MANUAL,
        AUTOMATIC
    };
    struct pump_t
    {
        TIM_TypeDef * const timer;
        const sr_io::out dir;
        const sr_io::out en;
        const sr_io::in err;
        motor* m;
    };
    extern pump_t instances[MOTORS_NUM];

    HAL_StatusTypeDef init();
    void deinit();
    void process();
    void log();

    void set_mode(mode_t v);
    void set_enable(bool v);
    void update_tunings();
}