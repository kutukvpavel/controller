#include "cli.h"

#include "nvs.h"

namespace cli_commands
{
    uint8_t hw_report(int argc, char** argv)
    {
        printf("HW report:\n"
            "\tADC modules present = %u\n"
            "\tDAC modules present = %u\n",
            adc::get_present_channels_count(), dac::get_present_modules_count());
        return EXIT_SUCCESS;
    }
    uint8_t nvs_dump(int argc, char** argv)
    {
        puts("ADC cals:");
        for (size_t i = 0; i < MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP; i++)
        {
            auto cal = nvs::get_adc_channel_cal(i);
            printf("\tADC channel #%u: gain = %f, offset = %f, invert = %u\n", 
                i, cal->k, cal->b, cal->invert);
        }
        puts("DAC cals:");
        for (size_t i = 0; i < MY_DAC_MAX_MODULES; i++)
        {
            auto cal = nvs::get_dac_cal(i);
            printf("\tDAC module #%u: gain = %f, offset = %f; i_gain = %f, i_offset = %f\n", 
                i, cal->k, cal->b, cal->current_k, cal->current_b);
        }
        puts("AIO cals:");
        for (size_t i = 0; i < a_io::in::INPUTS_NUM; i++)
        {
            auto cal = nvs::get_analog_input_cal(i);
            printf("\tAIO #%u: gain = %f, offset = %f\n", i, cal->k, cal->b);
        }
        puts("Motor params:");
        for (size_t i = 0; i < MOTORS_NUM; i++)
        {
            auto motor = nvs::get_motor_params(i);
            printf("\tMotor #%u: rate-to-speed = %f, teeth = %u, microsteps = %u, dir = %u, inv_en = %u, inv_err = %u\n", 
                i, motor->rate_to_speed, motor->teeth, motor->microsteps, motor->direction, motor->invert_enable, motor->invert_error);
        }
        auto ts_cal = nvs::get_temp_sensor_cal();
        printf("Temp sensor cal: gain = %f; offset = %f\n", ts_cal->k, ts_cal->b);
        return EXIT_SUCCESS;
    }
    uint8_t nvs_save(int argc, char** argv)
    {
        return nvs::save();
    }
    uint8_t me_toggle(int argc, char** argv)
    {
        LL_GPIO_TogglePin(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);
        return EXIT_SUCCESS;
    }

    uint8_t set_adc_cal(int argc, char** argv)
    {
        if (argc < 4) return EXIT_FAILURE;
        size_t index;
        if (sscanf(argv[1], "%u", &index) < 1) return EXIT_FAILURE;
        adc::ch_cal_t* buf = nvs::get_adc_channel_cal(index);
        if (sscanf(argv[2], "%f", &buf->k) < 1) return EXIT_FAILURE;
        if (sscanf(argv[3], "%f", &buf->b) < 1) return EXIT_FAILURE;
        return EXIT_SUCCESS;
    }
    uint8_t set_dac_cal(int argc, char** argv)
    {
        if (argc < 4) return EXIT_FAILURE;
        size_t index;
        if (sscanf(argv[1], "%u", &index) < 1) return EXIT_FAILURE;
        dac::cal_t* buf = nvs::get_dac_cal(index);
        if (sscanf(argv[2], "%f", &buf->k) < 1) return EXIT_FAILURE;
        if (sscanf(argv[3], "%f", &buf->b) < 1) return EXIT_FAILURE;
        return EXIT_SUCCESS;
    }
    uint8_t set_dac_current_cal(int argc, char** argv)
    {
        if (argc < 4) return EXIT_FAILURE;
        size_t index;
        if (sscanf(argv[1], "%u", &index) < 1) return EXIT_FAILURE;
        dac::cal_t* buf = nvs::get_dac_cal(index);
        if (sscanf(argv[2], "%f", &buf->current_k) < 1) return EXIT_FAILURE;
        if (sscanf(argv[3], "%f", &buf->current_b) < 1) return EXIT_FAILURE;
        return EXIT_SUCCESS;
    }
}

void my_cli_init(UART_HandleTypeDef* cli_uart)
{
    CLI_INIT(cli_uart);

    CLI_ADD_CMD("hw_report", "Report HW state", &cli_commands::hw_report);
    CLI_ADD_CMD("nvs_dump", "Dump NVS contents", &cli_commands::nvs_dump);
    CLI_ADD_CMD("nvs_save", "Save current calibrations and parameters to the NVS", &cli_commands::nvs_save);
    CLI_ADD_CMD("me_toggle", "Toggle Master Enable (/ME) pin", &cli_commands::me_toggle);

    CLI_ADD_CMD("set_adc_cal", "Set ADC calibration. Expects 3 args (index gain offset)", &cli_commands::set_adc_cal);
    CLI_ADD_CMD("set_dac_cal", "Set DAC voltage calibration. Expects 3 args (index gain offset)", &cli_commands::set_dac_cal);
    CLI_ADD_CMD("set_dac_current_cal", "Set DAC current calibration. Expects 3 args (index gain offset)", &cli_commands::set_dac_current_cal);

    DBG("Goodnight Moon!");
}