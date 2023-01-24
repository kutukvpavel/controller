/**
 * @file average.h
 * @author Kutukov Pavel
 * @brief Temporary filter implementation as a moving average (TODO: hardware FFT)
 * @version 0.1
 * @date 2022-03-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "user.h"

class average
{
private:
    float* buffer;
    size_t buffer_head;
    float accumulator;
    size_t current_value_count;
    size_t window_length;
    size_t buffer_length;

public:
    average(size_t window);
    float get_average();
    float get_last_added_value();
    void enqueue(float val);
    void clear();
    ~average();
};
