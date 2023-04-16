#include "pumps.h"

#include <math.h>
#include "commands.h"
#include "../pid/PID_v1.h"

float setpoint;
float input;
float output;
PID regulator_instance(&input, &output, &setpoint, 0, 0, 0, PID_DIR_DIRECT, HAL_GetTick);

namespace pumps
{
    pump_t instances[MOTORS_NUM] =
    {
        {
            .timer = TIM9,
            .dir = sr_io::out::MOTOR_DIR0,
            .en = sr_io::out::MOTOR_EN,
            .err = sr_io::in::MOTOR_ERR0
        },
        {
            .timer = TIM10,
            .dir = sr_io::out::MOTOR_DIR1,
            .en = sr_io::out::MOTOR_EN,
            .err = sr_io::in::MOTOR_ERR1
        },
        {
            .timer = TIM11,
            .dir = sr_io::out::MOTOR_DIR2,
            .en = sr_io::out::MOTOR_EN,
            .err = sr_io::in::MOTOR_ERR2
        }
    };
    PACKED_FOR_MODBUS const params_t* params;

    HAL_StatusTypeDef init()
    {
        static_assert(sizeof(params_t) % sizeof(float) == 0);

        for (size_t i = 0; i < MOTORS_NUM; i++)
        {
            auto& item = instances[i];
            item.m = new motor(item.timer, cmd::get_motor_params(i), item.err, item.en, item.dir);
            assert_param(item.m);
        }
        params = cmd::get_pump_params();
        regulator_instance.SetOutputLimits(0, 1);
        regulator_instance.SetSampleTime(200);
        update_tunings();
        return HAL_OK;
    }
    void deinit()
    {
        for (size_t i = 0; i < MOTORS_NUM; i++)
        {
            delete instances[i].m;
        }
    }
    void compute_pid()
    {
        if (regulator_instance.Compute() && (regulator_instance.GetMode() == PID_MODE_AUTOMATIC))
        {
            assert_param(isfinite(output));

            //Output is a ratio of flow rates
            instances[params->high_concentration_motor_index].m->set_volume_rate(params->total_flowrate * output);
            instances[params->low_concentration_motor_index].m->set_volume_rate(params->total_flowrate * (1 - output));
        }
    }
    void process()
    {
        set_mode(cmd::get_status_bit_set(MY_CMD_STATUS_REGULATE) ? pumps::mode_t::AUTOMATIC : pumps::mode_t::MANUAL);
        update_tunings();
        set_concentration_setpoint(cmd::get_regulator_setpoint());
        set_enable(cmd::get_status_bit_set(MY_CMD_STATUS_MOTOR_EN));
    }

    void set_enable(bool v)
    {
        for (size_t i = 0; i < MOTORS_NUM; i++)
        {
            auto& item = instances[i];
            item.m->set_enable(v);
        }
    }
    void set_mode(mode_t v)
    {
        if (v == mode_t::AUTOMATIC)
        {
            regulator_instance.SetMode(PID_MODE_AUTOMATIC);
        }
        else
        {
            regulator_instance.SetMode(PID_MODE_MANUAL);
        }
    }
    void set_concentration_setpoint(float v)
    {
        setpoint = v;
    }
    void set_concentration_feedback(float v)
    {
        input = v;
    }
    void update_tunings()
    {
        regulator_instance.SetTunings(params->kP, params->kI, params->kD);
    }
    size_t get_sensing_adc_channel()
    {
        return params->sensing_adc_channel_index;
    }
}
