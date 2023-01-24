#pragma once

namespace sr_io
{
    enum in
    {
        MOTOR_ERR0,
        MOTOR_ERR1,
        MOTOR_ERR2,
        IN3,
        IN4,
        IN5,
        IN6,
        IN7,

        INPUT_NUM
    };
    enum out
    {
        ADC_RES,
        ADC_EN,
        DAC_RES,
        DAC_EN,
        MOTOR_EN,
        MOTOR_DIR0,
        MOTOR_DIR1,
        MOTOR_DIR2,
        OUT0,
        OUT1,
        OUT2,
        OUT3,
        OUT4,
        OUT5,
        OUT6,
        OUT7,

        OUTPUT_NUM
    };

    bool get_input(in i);
    void set_output(out i, bool v);

    void sync();
}