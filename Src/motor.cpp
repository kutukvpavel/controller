#include "motor.h"

#include <math.h>

void motor::set_speed(float v)
{
    float target_period = roundf(clk / (cal->microsteps * cal->teeth * v)) - 1;
    if (target_period > UINT16_MAX) target_period = UINT16_MAX;
    else if (target_period < MOTOR_TIMER_MIN_VALUE) target_period = MOTOR_TIMER_MIN_VALUE;
    tim->ARR = static_cast<uint16_t>(target_period);
}

void motor::set_volume_rate(float v)
{
    set_speed(cal->rate_to_speed * v);
}

bool motor::get_error()
{
    bool res = sr_io::get_input(err_input);
    if (cal->invert_error) res = !res;
    return res;
}
void motor::set_enable(bool v)
{
    sr_io::set_output(en_output, cal->invert_enable ? !v : v);
}

motor::motor(TIM_TypeDef* t, motor_params_t* c, sr_io::in err, sr_io::out en)
{
    tim = t;
    cal = c;
    clk = static_cast<float>(HAL_RCC_GetSysClockFreq()) / tim->PSC;
    err_input = err;
    en_output = en;
}
motor::~motor()
{
}