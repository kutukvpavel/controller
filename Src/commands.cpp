#include "commands.h"

void clear_stream(user::Stream* stream)
{
    char c;
    while ((c = stream->read()) != '\0') if (c == '\n') break;
}

namespace cmd
{
    void process(user::Stream* stream)
    {
        if (!stream->available()) return;
        char c = stream->read();
        switch (c)
        {
        case 'A':
            user::status ^= MY_STATUS_ACQUIRE;
            break;
        default:
            break;
        }
        clear_stream(stream);
    }
}