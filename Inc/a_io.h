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

    void init(in_cal_t* cal);

    float get_input(in i);
} // namespace a_io
