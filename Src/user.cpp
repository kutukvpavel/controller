#include "usbd_cdc_if.h"
#include "adc_modules.h"
#include "dac_modules.h"
#include "user.h"

//Private vars
char output_buf[256];
static user::pin_t led_pin = user::pin_t(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);
user::pin_t cs_pin = { nCS_GPIO_Port, nCS_Pin };
static user::Stream* cdc_stream = new user::Stream();

/**
 * PRIVATE
 */

void cdc_transmit_blocking(uint8_t* buf, uint16_t len)
{
    while (CDC_Can_Transmit() != USBD_OK);
    CDC_Transmit_FS(buf, len);
}

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

static void send_output(size_t len)
{
    cdc_transmit_blocking(reinterpret_cast<uint8_t*>(output_buf), static_cast<uint16_t>(len));
}

static void wait_for_input()
{
    user_usb_prints("Send anything to continue...\n");
    while (!cdc_stream->available()) LL_mDelay(10);
    while (cdc_stream->read() != '\0') LL_mDelay(1);
    user_usb_prints("Starting execution.\n");
}

/**
 * PUBLIC, hide behind a namespace
 */

namespace user 
{
    /****
     * MAIN
     * */
    void setup(SPI_HandleTypeDef* adc_spi, SPI_HandleTypeDef* dac_spi, I2C_HandleTypeDef* dac_i2c)
    {
        while (CDC_IsConnected() != USBD_OK); //Note: requires DTR (i.e. hardware handshake)
        CDC_Register_RX_Callback(cdc_receive);
        user_usb_prints("Hello World!\n");
        wait_for_input();

        //ADC
        /*adc::init(adc_spi);
        LL_mDelay(1000); //Allow the boards to power up
        user_usb_prints("Probing ADC modules...\n");
        adc::probe();
        send_output(adc::dump_module_report(output_buf, sizeof(output_buf)));
        wait_for_input();*/

        //DAC
        dac::init(dac_spi, &cs_pin, dac_i2c);
        LL_mDelay(1000); //Allow the boards to power up
        user_usb_prints("Probing DAC modules...\n");
        dac::probe();
        send_output(dac::dump_module_report(output_buf, sizeof(output_buf)));
        wait_for_input();

        //Last preparations
        adc::increment_and_sync();
        dac::set_all(1);
    }
    void main()
    {
        //ADC
        /*adc::drdy_check();
        if (adc::status == MY_ADC_STATUS_READ_PENDING) 
        {
            adc::read();
            adc::increment_and_sync();
            send_output(adc::dump_last_data(output_buf, sizeof(output_buf)));
        }*/

        //DAC
        dac::read_current();
        dac::correct_for_current();
        send_output(dac::dump_last_currents(output_buf, sizeof(output_buf)));

        //Heartbeat
        LL_GPIO_TogglePin(led_pin.port, led_pin.mask);
        user_usb_prints("Cycle completed.\n");
        wait_for_input();
    }

    //Compatibility API
    bool Stream::available()
    {
        supervise_indexes(&_tail, &_head);
        return _tail != _head;
    }
    uint8_t Stream::read()
    {
        if (!available()) return '\0';
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
        return len;
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