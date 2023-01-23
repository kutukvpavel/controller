#pragma once

#include "user.h"

class motor
{
private:
    /* data */
public:
    motor(TIM_HandleTypeDef);
    ~motor();
};

motor::motor(TIM_HandleTypeDef t)
{
    
}

motor::~motor()
{
}
