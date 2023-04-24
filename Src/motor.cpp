#include "motor.h"

#include <math.h>

void motor::set_speed(float v)
{
    assert_param(isfinite(v));
    
    float target_period = roundf(clk / (cal->microsteps * cal->teeth * v)) - 1;
    if (target_period > UINT16_MAX) target_period = UINT16_MAX;
    else if (target_period < MOTOR_TIMER_MIN_VALUE) target_period = MOTOR_TIMER_MIN_VALUE;
    tim->ARR = static_cast<uint16_t>(target_period);
    //DBG("Computed ARR register: %lu", tim->ARR);
    last_set_speed = v;
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
float motor::get_speed()
{
    return last_set_speed;
}
float motor::get_volume_rate()
{
    assert_param(cal->rate_to_speed > 0);
    return last_set_speed / cal->rate_to_speed;
}

motor::motor(TIM_TypeDef* t, motor_params_t* c, sr_io::in err, sr_io::out en, sr_io::out dir)
{
    static_assert(sizeof(motor_params_t) % sizeof(float) == 0);

    tim = t;
    cal = c;
    clk = static_cast<float>(HAL_RCC_GetSysClockFreq()) / tim->PSC;
    err_input = err;
    en_output = en;
    sr_io::set_output(dir, c->direction);
    
}
motor::~motor()
{
}