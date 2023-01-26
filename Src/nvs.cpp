#include "nvs.h"

namespace nvs
{
    motor_params_t motor_defaults[MOTORS_NUM] =
    {
        {
            
        },
        {

        },
        {

        }
    };

    const motor_params_t* get_motor_params(size_t i)
    {
        return &(motor_defaults[i]);
    }
} // namespace nvs
