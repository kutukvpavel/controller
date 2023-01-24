#pragma once

#include "user.h"
#include "sr_io.h"

#include <stdint.h>

struct motor_params_t
{
    float rate_to_speed; //Characterizes tubing
    uint8_t microsteps;
    uint16_t teeth; //Usually 200 (1.8 deg stepper)
    bool invert_enable;
    bool invert_error;
};

class motor
{
private:
    TIM_TypeDef* tim;
    motor_params_t* cal;
    float clk;
    sr_io::in err_input;
    sr_io::out en_output;
public:
    motor(TIM_TypeDef*, motor_params_t*, sr_io::in, sr_io::out);

    /// @brief 
    /// @param v Speed in rotations per second (RPS, Hz)
    void set_speed(float v);
    /// @brief 
    /// @param v Flowrate in ???
    void set_volume_rate(float v);

    void set_enable(bool v);
    bool get_error();

    ~motor();
};
