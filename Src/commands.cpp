#include "commands.h"

void clear_stream(user::Stream* stream)
{
    char c;
    while ((c = stream->read()) != '\0') if (c == '\n') break;
}

namespace cmd
{
    float dac_setpoint = 1;

    void report_ready()
    {
        user_usb_prints("READY...\n");
    }

    void process(user::Stream* stream)
    {
        if (!stream->available()) return;
        char c = stream->read();
        user_usb_prints("PARSED.\n");
        char buf[16];
        switch (c)
        {
        case 'A':
            if (user::status & MY_STATUS_ACQUIRE)
            {
                user_usb_prints("END.\n");
            }
            else
            {
                user_usb_prints("ACQ.\n");
            }
            user::status ^= MY_STATUS_ACQUIRE;
            break;
        case 'D':
            stream->readBytes(reinterpret_cast<uint8_t*>(buf), 8);
            sscanf(buf, "%f", &dac_setpoint);
            user::status |= MY_STATUS_SETPOINT_CHANGED;
            break;
        default:
            break;
        }
        clear_stream(stream);
    }
}