#pragma once

#include "user.h"
#include "sr_io.h"

#include <stdint.h>

#define MOTORS_NUM 3

struct PACKED_FOR_MODBUS motor_params_t
{
    float rate_to_speed; //Characterizes tubing
    uint16_t microsteps;
    uint16_t teeth; //Usually 200 (1.8 deg stepper)
    uint16_t invert_enable;
    uint16_t invert_error;
    uint16_t direction;
};

class motor
{
private:
    TIM_TypeDef* tim;
    motor_params_t* cal;
    float clk;
    sr_io::in err_input;
    sr_io::out en_output;
    float last_set_speed = 0;
public:
    motor(TIM_TypeDef* t, motor_params_t* c, sr_io::in err, sr_io::out en, sr_io::out dir);

    /// @brief 
    /// @param v Speed in rotations per second (RPS, Hz)
    void set_speed(float v);
    /// @brief 
    /// @param v Flowrate in ???
    void set_volume_rate(float v);

    void set_enable(bool v);
    bool get_error();
    float get_speed();

    ~motor();
};
