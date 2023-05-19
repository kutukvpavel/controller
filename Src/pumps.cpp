#include "pumps.h"

#include <math.h>
#include "commands.h"
#include "../pid/PID_v1.h"
#include "my_math.h"


static float setpoint;
static float input = 0;
static float output = 0;
static float ideal_mix_ratio = 0;
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
        regulator_instance.SetControllerDirection(PID_DIR_REVERSE);
        regulator_instance.SetMaxITerm(0.5);
        regulator_instance.SetSampleTime(500);
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
    void process()
    {
        const pumps::params_t *reg_params = cmd::get_pump_params();
        const adc::channel_t *conc_sense_ch = adc::get_channel(reg_params->sensing_adc_channel_index);
        const adc::channel_t *adc_high_ch = adc::get_channel(reg_params->high_conc_adc_ch_index);
        const adc::channel_t *adc_low_ch = adc::get_channel(reg_params->low_conc_adc_ch_index);
        const dac::module_t *dac_high_ch = &(dac::modules[reg_params->high_concentration_dac_channel_index]);
        const dac::module_t *dac_low_ch = &(dac::modules[reg_params->low_concentration_dac_channel_index]);
        size_t present_dac_modules_num = dac::get_present_modules_count();

        // Sanity check
        if (!conc_sense_ch || !adc_high_ch || !adc_low_ch ||
            !(reg_params->high_concentration_dac_channel_index < present_dac_modules_num) ||
            !(reg_params->low_concentration_dac_channel_index < present_dac_modules_num) ||
            !isfinite(output))
        {
            cmd::reset_status_bit(MY_CMD_STATUS_REGULATE);
            return;
        }
        assert_param(isfinite(input));

        //Calculate state and PID
        bool enable = cmd::get_status_bit_set(MY_CMD_STATUS_MOTOR_EN);
        set_enable(enable);
        bool automatic = cmd::get_status_bit_set(MY_CMD_STATUS_REGULATE) && enable;
        set_mode(automatic ? pumps::mode_t::AUTOMATIC : pumps::mode_t::MANUAL);
        if (automatic)
        {
            float sense_p = my_math::volts_to_partial_pressure(conc_sense_ch->averaging_container->get_average(), cmd::get_furnace_temperature());
            float adc_high_p = my_math::volts_to_partial_pressure(adc_high_ch->averaging_container->get_average(), cmd::get_furnace_temperature());
            float adc_low_p = my_math::volts_to_partial_pressure(adc_high_ch->averaging_container->get_average(), cmd::get_furnace_temperature());
            if (isfinite(sense_p) && isfinite(adc_high_p) && isfinite(adc_low_p)) // False if averaging container has no points yet
            {
                input = sense_p;
                ideal_mix_ratio = (adc_high_p - setpoint) / (adc_high_p - adc_low_p);
                /*if (ideal_mix_ratio < 0) ideal_mix_ratio = 0;
                else if (ideal_mix_ratio > 1) ideal_mix_ratio = 1;*/
                regulator_instance.Compute();
                float corrected_ratio = ideal_mix_ratio + output;
                if (corrected_ratio < 0) corrected_ratio = 0;
                else if (corrected_ratio > 1) corrected_ratio = 1;
                instances[params->high_concentration_motor_index].m->set_volume_rate(params->total_flowrate * (1 - corrected_ratio));
                instances[params->low_concentration_motor_index].m->set_volume_rate(params->total_flowrate * corrected_ratio);
                for (size_t i = 0; i < MOTORS_NUM; i++)
                {
                    cmd::set_motor_speed(instances[i].m->get_volume_rate(), i);
                }
            }
        }

        //Update modbus registers
        update_tunings();
        setpoint = cmd::get_regulator_setpoint();
        if (!automatic)
        {
            for (size_t i = 0; i < MOTORS_NUM; i++)
            {
                instances[i].m->set_volume_rate(cmd::get_motor_speed(i));
            }
        }
    }
    void log()
    {
        printf("Ideal r = %5.3f, PID input = %5.2e\n", ideal_mix_ratio, input);
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
    void update_tunings()
    {
        regulator_instance.SetTunings(params->kP, params->kI, params->kD);
    }
}
