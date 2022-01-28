#include "usbd_cdc_if.h"
#include "user.h"

//Private vars
static int delay_length = 500;
static user::pin_t led_pin = user::pin_t(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);
static user::Stream* cdc_stream = new user::Stream();

//Private forward-decls
static void cdc_receive(uint8_t* buf, uint32_t* len);
static void supervise_indexes(uint16_t* _tail, uint16_t* _head);

/**
 * PUBLIC, hide behind a namespace
 */

namespace user 
{
    /****
     * MAIN
     * */
    void setup()
    {
        while (CDC_IsConnected() != USBD_OK); //Note: requires DTR (i.e. hardware handshake)
        CDC_Register_RX_Callback(cdc_receive);
        user_prints("Hello World!\n");
    }
    void main()
    {
        LL_GPIO_TogglePin(led_pin.port, led_pin.mask);
        LL_mDelay(delay_length);
    }

    //API
    bool Stream::available()
    {
        supervise_indexes(&_tail, &_head);
        return _tail != _head;
    }
    uint8_t Stream::read()
    {
        uint8_t r = _buffer[_head++];
        supervise_indexes(&_tail, &_head);
        return r;
    }
    uint16_t Stream::readBytes(uint8_t* buf, uint16_t len)
    {
        len = min(len, _tail - _head);
        for (size_t i = 0; i < len; i++)
        {
            buf[i] = _buffer[_head++];
        }
        supervise_indexes(&_tail, &_head);
    }
    uint16_t Stream::write(uint8_t* buf, uint16_t len)
    {
        if (CDC_Transmit_FS(buf, len) == USBD_OK) return len;
        return 0;
    }
    uint16_t Stream::availableForWrite()
    {
        return CDC_Can_Transmit() ? SERIAL_BUFFER_SIZE : 0;
    }
    void Stream::receive(uint8_t* buf, uint16_t len)
    {
        supervise_indexes(&_tail, &_head);
        len = min(len, array_size(_buffer) - _tail);
        for (size_t i = 0; i < len; i++)
        {
            _buffer[_tail++] = buf[i];
        }
    }

    uint32_t micros()
    {
        return MY_TIM_MICROS->CNT;
    }
    void digitalWrite(pin_t p, uint8_t state)
    {
        if (state) LL_GPIO_SetOutputPin(p.port, p.mask);
        else LL_GPIO_ResetOutputPin(p.port, p.mask);
    }
    void pinMode(pin_t p, uint8_t mode)
    {
        if (mode == OUTPUT) LL_GPIO_SetPinMode(p.port, p.mask, LL_GPIO_MODE_OUTPUT);
    }
    int min(int x, int y)
    {
        return x < y ? x : y;
    }
    uint16_t word(uint8_t h, uint8_t l)
    {
        return static_cast<uint16_t>(h) << 8 | l;
    }
}

/**
 * PRIVATE
 */

static void supervise_indexes(uint16_t* _tail, uint16_t* _head)
{
    if (*_tail == *_head)
    {
        *_tail = 0;
        *_head = 0;
    }
}

static void cdc_receive(uint8_t* buf, uint32_t* len)
{
    cdc_stream->receive(buf, static_cast<uint16_t>(*len));
}