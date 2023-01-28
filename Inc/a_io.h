#pragma once

#include "user.h"

namespace a_io
{
    struct in_cal_t
    {
        float k;
        float b;
    };

    enum in
    {
        A0,
        A1,
        A2,
        A3,

        INPUTS_NUM
    };

    extern float voltages[in::INPUTS_NUM];
    extern float temperature;

    void init(ADC_HandleTypeDef* adc, const in_cal_t* cal, const in_cal_t* temp_cal);
    void poll();
} // namespace a_io
