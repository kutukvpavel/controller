#include "user.h"

#include "usbd_cdc_if.h"
#include "commands.h"
#include "adc_modules.h"
#include "dac_modules.h"
#include "sr_io.h"
#include "a_io.h"
#include "nvs.h"
#include "../ModbusPort/src/ModbusSlave.h"

#define SR_SYNC_INTERVAL 50 // mS

// Private vars
char output_buf[256];
user::pin_t cs_pin = {nCS_GPIO_Port, nCS_Pin};
static uint8_t zero_arr[1] = {0};
static user::pin_t led_pin = user::pin_t(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);
static user::Stream cdc_stream = user::Stream();

/**
 * ISRs
 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    if (cmd::get_status_bit_set(MY_CMD_ACQUIRE))
        cmd::set_status_bit(MY_CMD_STATUS_HAVE_NEW_DATA);
    LL_GPIO_TogglePin(led_pin.port, led_pin.mask);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 0.1S timer: grab INA219 current data, and depolarize
    {
        //user::status |= MY_STATUS_DEPOLARIZE;
    }
    if (htim->Instance == TIM4) // 0.1S+X offset timer for PWM, stop depolarization and correct DAC
    {
        user::status |= MY_STATUS_CORRECT_DAC;
    }
}

/**
 * PRIVATE
 */

static void supervise_indexes(uint16_t *_tail, uint16_t *_head)
{
    if (*_tail == *_head)
    {
        *_tail = 0;
        *_head = 0;
    }
}

static void cdc_receive(uint8_t *buf, uint32_t *len)
{
    cdc_stream.receive(buf, static_cast<uint16_t>(*len));
}

static void dbg_wait_for_input()
{
#if DEBUG_STEP_BY_STEP
    puts("Send anything to continue...\n");
    while (!cdc_stream->available())
        LL_mDelay(10);
    while (cdc_stream->read() != '\0')
        LL_mDelay(1);
    puts("Starting execution.\n");
#endif
}

/**
 * PUBLIC, hide behind a namespace
 */

namespace user
{
    volatile uint8_t status;

    /****
     * MAIN
     * */
    void setup(SPI_HandleTypeDef *adc_spi, SPI_HandleTypeDef *dac_spi, I2C_HandleTypeDef *dac_i2c, ADC_HandleTypeDef *adc)
    {
        while (CDC_IsConnected() != USBD_OK); // Note: requires DTR (i.e. hardware handshake)
        CDC_Register_RX_Callback(cdc_receive);
        nvs::init(dac_i2c);
        cmd::init(cdc_stream);
        a_io::init(adc, nvs::get_analog_input_cal(0), nvs::get_temp_sensor_cal());
        puts("Hello World!\n");
        dbg_wait_for_input();

        // ADC
        HAL_SPI_Transmit(adc_spi, zero_arr, 1, 100); // Get SPI pins into an approptiate idle state before any /CS is asserted (SPI_MspInit doesn't do that FSR)
        adc::init(adc_spi, &cs_pin, nvs::get_adc_channel_cal(0));
        LL_mDelay(1000); // Allow the boards to power up
        puts("Probing ADC modules...\n");
        adc::probe();
        //send_output(adc::dump_module_report(output_buf, sizeof(output_buf)));
        dbg_wait_for_input();

        // DAC
        HAL_SPI_Transmit(dac_spi, zero_arr, 1, 100); // Get SPI pins into an approptiate idle state before any /CS is asserted (SPI_MspInit doesn't do that FSR)
        dac::init(dac_spi, &cs_pin, dac_i2c, nvs::get_dac_cal(0));
        LL_mDelay(1000); // Allow the boards to power up
        puts("Probing DAC modules...\n");
        dac::probe();
        //send_output(dac::dump_module_report(output_buf, sizeof(output_buf)));
        dbg_wait_for_input();

        // Last preparations
        LL_mDelay(1000);
        cmd::report_ready();
        adc::increment_and_sync();
    }
    void main()
    {
        static uint32_t tick;
        static uint32_t last_gpio_sync = 0;

        tick = HAL_GetTick();

        // GPIO
        if (tick - last_gpio_sync > SR_SYNC_INTERVAL)
        {
            sr_io::sync();
            last_gpio_sync = tick;
        }
        a_io::poll();

        // PC commands
        cmd::poll();
        // cmd::process(&cdc_stream, output_buf, sizeof(output_buf));

        // Data output
        /*if (status & MY_STATUS_DUMP_DATA)
        {
            send_output(adc::dump_last_data(output_buf, sizeof(output_buf)));
            send_output(dac::dump_last_data(output_buf, sizeof(output_buf)));
            send_output(cmd::report_depolarization_percent(output_buf, size_t(output_buf)));

            status &= ~MY_STATUS_DUMP_DATA;
        }*/

        // ADC
        adc::drdy_check();
        if (adc::status == MY_ADC_STATUS_READ_PENDING)
        {
            adc::read();
            adc::increment_and_sync();
        }

        // DAC
        /*if (status & MY_STATUS_DEPOLARIZE)
        {
            dac::read_current();
            if (cmd::get_depolarization_percent() > 0)
                dac::start_depolarization();
            status &= ~MY_STATUS_DEPOLARIZE;
        }*/
        if (status & MY_STATUS_CORRECT_DAC)
        {
            dac::stop_depolarization();
            if (cmd::get_status_bit_set(MY_CMD_STATUS_DAC_CORRECT))
            {
                dac::correct_for_current(); // Single-shot
                cmd::reset_status_bit(MY_CMD_STATUS_DAC_CORRECT);
            }
            status &= ~MY_STATUS_CORRECT_DAC;
        }

        // Heartbeat
        puts("Cycle completed.\n");
        dbg_wait_for_input();
    }

    // Compatibility API
    bool Stream::available()
    {
        supervise_indexes(&_tail, &_head);
        return _tail != _head;
    }
    bool Stream::new_line_reached()
    {
        bool tmp = _newline_present;
        if (tmp)
            _newline_present = false;
        return tmp;
    }
    uint8_t Stream::read()
    {
        if (!available())
            return '\0';
        uint8_t r = _buffer[_head++];
        supervise_indexes(&_tail, &_head);
        return r;
    }
    uint16_t Stream::readBytes(uint8_t *buf, uint16_t len)
    {
        len = min(len, _tail - _head);
        for (size_t i = 0; i < len; i++)
        {
            buf[i] = _buffer[_head++];
        }
        buf[len] = '\0';
        supervise_indexes(&_tail, &_head);
        return len;
    }
    uint16_t Stream::write(uint8_t *buf, uint16_t len)
    {
        while (CDC_Can_Transmit() != USBD_OK);
        if (CDC_Transmit_FS(buf, len) == USBD_OK)
            return len;
        return 0;
    }
    uint16_t Stream::availableForWrite()
    {
        return CDC_Can_Transmit() ? SERIAL_BUFFER_SIZE : 0;
    }
    void Stream::receive(uint8_t *buf, uint16_t len)
    {
        supervise_indexes(&_tail, &_head);
        len = min(len, array_size(_buffer) - _tail);
        for (size_t i = 0; i < len; i++)
        {
            _buffer[_tail++] = buf[i];
        }
        for (size_t i = 0; i < len; i++)
        {
            if (buf[i] == '\n')
            {
                _newline_present = true;
                break;
            }
        }
    }

    uint32_t micros()
    {
        return MY_TIM_MICROS->CNT;
    }
    void uDelay(uint32_t us)
    {
        uint32_t start = micros();
        while ((micros() - start) < us)
            ;
    }
    void digitalWrite(pin_t p, uint8_t state)
    {
        if (state)
            LL_GPIO_SetOutputPin(p.port, p.mask);
        else
            LL_GPIO_ResetOutputPin(p.port, p.mask);
    }
    void pinMode(pin_t p, uint8_t mode)
    {
        if (mode == OUTPUT)
            LL_GPIO_SetPinMode(p.port, p.mask, LL_GPIO_MODE_OUTPUT);
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