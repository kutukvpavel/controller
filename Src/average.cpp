#include "average.h"

average::average(size_t window)
{
    buffer_length = window + 1;
    buffer = new float[buffer_length]();
    window_length = window;
    accumulator = 0;
    current_value_count = 0;
    buffer_head = 0;
}

float average::get_last_added_value()
{
    return buffer[(buffer_head - 1u) % buffer_length];
}

float average::get_average()
{
    if (current_value_count > window_length) current_value_count = window_length;
    return accumulator / current_value_count;
}

void average::enqueue(float val)
{
    buffer[buffer_head] = val;
    buffer_head = (buffer_head + 1u) % buffer_length;
    accumulator += val - buffer[buffer_head];
    current_value_count++;
}

void average::clear()
{
    current_value_count = 0;
    buffer_head = 0;
    accumulator = 0;
    for (size_t i = 0; i < window_length; i++)
    {
        buffer[i] = 0;
    }
}

average::~average()
{
    delete[] buffer;
}