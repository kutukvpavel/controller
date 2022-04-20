#include "commands.h"
#include "dac_modules.h"
#include "adc_modules.h"
#include <math.h>

void clear_stream(user::Stream* stream)
{
    char c;
    while ((c = stream->read()) != '\0') if (c == '\n') break;
}

void read_float(user::Stream* stream, float* val)
{
    char buf[9];
    stream->readBytes(reinterpret_cast<uint8_t*>(buf), 8);
    sscanf(buf, "%f", val);
}

namespace cmd
{
    uint8_t status = 0;
    float dac_setpoint = 0.2;
    float depolarization_percent = 0;
    float depolarization_setpoint = 0;

    void report_ready()
    {
        user_usb_prints("READY...\n");
    }

    size_t report_depolarization_percent(char* output_buf, size_t max_len)
    {
        return snprintf(output_buf, max_len, DEPOLARIZATION_REPORT_FORMAT, depolarization_percent);
    }

    void process(user::Stream* stream, char* output_buf, size_t max_len)
    {
        if (!stream->available()) return;
        if (!stream->new_line_reached()) return;
        char c = stream->read();
        user_usb_prints("PARSED.\n");
        switch (c)
        {
        case 'A':
            if (status & MY_CMD_ACQUIRE)
            {
                user_usb_prints("END.\n");
            }
            else
            {
                user_usb_prints("ACQ.\n");
            }
            status ^= MY_CMD_ACQUIRE;
            break;
        case 'S':
            read_float(stream, &dac_setpoint);
            dac::set_all(dac_setpoint);
            break;
        case 'P':
        {
            float temp;
            read_float(stream, &temp);
            if (temp > 1) temp = 1;
            depolarization_percent = temp;
            if (depolarization_percent < 0.01) depolarization_percent = 0;
            if (temp < 0.01) temp = 0.01; //So that the timer doesn't fire constantly
            MY_TIM_DEPOLARIZATION->ARR = static_cast<uint16_t>(roundf(temp * 30000));
            break;
        }
        case 'D': //Setting to DAC setpoint disables depolarization
            read_float(stream, &depolarization_setpoint);
            dac::set_depolarization(depolarization_setpoint);
            break;
        case 'R':
            while (CDC_Can_Transmit() != HAL_OK); //Ensure the PARSED message gets transmitted
            HAL_NVIC_SystemReset();
            break;
        case 'E':
            status |= MY_CMD_STATUS_DAC_CORRECT; //Single-shot
            break;
        case 'I':
        {
            size_t len = adc::dump_module_report(output_buf, max_len);
            cdc_transmit_blocking(reinterpret_cast<uint8_t*>(output_buf), len);
            len = dac::dump_module_report(output_buf, max_len);
            cdc_transmit_blocking(reinterpret_cast<uint8_t*>(output_buf), len);
            break;
        }
        default:
            break;
        }
        clear_stream(stream);
    }
}