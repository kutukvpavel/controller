#include "commands.h"
#include "dac_modules.h"
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

void supervise_depolarization()
{
    if (cmd::depolarization_setpoint == cmd::dac_setpoint) cmd::depolarization_setpoint = NAN;
}

namespace cmd
{
    uint8_t status = MY_CMD_DAC_SETPOINT_CHANGED;
    float dac_setpoint = 0.2;
    uint16_t depolarization_time = 10000;
    float depolarization_setpoint = 0;

    void report_ready()
    {
        user_usb_prints("READY...\n");
    }

    void process(user::Stream* stream)
    {
        if (!stream->available()) return;
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
            supervise_depolarization();
            dac::set_all(dac_setpoint);
            break;
        case 'P':
        {
            float temp;
            read_float(stream, &temp);
            if (temp > 1) temp = 1;
            if (temp < 0.01) temp = 0.01;
            depolarization_time = static_cast<uint16_t>(roundf(temp * 30000));
            MY_TIM_DEPOLARIZATION->ARR = depolarization_time;
            break;
        }
        case 'D':
            read_float(stream, &depolarization_setpoint);
            supervise_depolarization();
            dac::set_depolarization(depolarization_setpoint);
            break;
        case 'R':
            while (CDC_Can_Transmit() != HAL_OK); //Ensure the PARSED message gets transmitted
            HAL_NVIC_SystemReset();
            break;
        case 'C':
            status ^= MY_CMD_STATUS_DAC_CORRECTION;
            break;
        default:
            break;
        }
        clear_stream(stream);
    }
}