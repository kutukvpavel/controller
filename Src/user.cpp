#include "user.h"

#include "usbd_cdc_if.h"
#include "commands.h"
#include "adc_modules.h"
#include "dac_modules.h"
#include "a_io.h"
#include "pumps.h"
#include "my_math.h"
#include "../ModbusPort/src/ModbusSlave.h"
#include <math.h>

#define SR_SYNC_INTERVAL 50 // mS
#define REGULATOR_UPDATE_INTERVAL 500 //mS

// Private vars
user::pin_t cs_pin = user::pin_t(nCS_GPIO_Port, nCS_Pin);
static uint8_t zero_arr[1] = {0};
static user::pin_t me_pin = user::pin_t(MASTER_ENABLE_GPIO_Port, MASTER_ENABLE_Pin);
static user::Stream cdc_stream = user::Stream();
static volatile bool dump_data = false;

/**
 * ISRs
 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    if (cmd::get_status_bit_set(MY_CMD_ACQUIRE))
    {
        dump_data = true;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) // 0.1S timer
    {
        
    }
    if (htim->Instance == TIM4) // 0.1S+X offset timer
    {
        
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
    while (getchar() == EOF)
        LL_mDelay(10);
    while (getchar() != EOF)
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
    void setup(SPI_HandleTypeDef *adc_spi, SPI_HandleTypeDef *dac_spi, I2C_HandleTypeDef *dac_i2c, ADC_HandleTypeDef *adc,
        UART_HandleTypeDef *console_uart)
    {
        my_cli_init(console_uart);

        CDC_Register_RX_Callback(cdc_receive);
        cmd::init(cdc_stream, dac_i2c);
        a_io::init(adc, cmd::get_analog_input_cal(0), cmd::get_temp_sensor_cal());
        LL_GPIO_ResetOutputPin(me_pin.port, me_pin.mask);
        dbg_wait_for_input();

        // ADC
        HAL_SPI_Transmit(adc_spi, zero_arr, 1, 100); // Get SPI pins into an approptiate idle state before any /CS is asserted (SPI_MspInit doesn't do that FSR)
        adc::init(adc_spi, &cs_pin, cmd::get_adc_channel_cal(0));
        LL_mDelay(1000); // Allow the boards to power up
        DBG("Probing ADC modules...");
        adc::probe();
        cmd::set_adc_channels_present(adc::get_present_channels_count());
        adc::dump_module_report();
        dbg_wait_for_input();

        // DAC
        HAL_SPI_Transmit(dac_spi, zero_arr, 1, 100); // Get SPI pins into an approptiate idle state before any /CS is asserted (SPI_MspInit doesn't do that FSR)
        dac::init(dac_spi, &cs_pin, dac_i2c, cmd::get_dac_cal(0));
        LL_mDelay(1000); // Allow the boards to power up
        DBG("Probing DAC modules...");
        dac::probe();
        cmd::set_dac_channels_present(dac::get_present_modules_count());
        dac::dump_module_report();
        dbg_wait_for_input();

        //Pumps
        DBG("Initializing pumps...");
        if (pumps::init() == HAL_OK)
        {
            DBG("\tPumps initialized successfully.");
        }
        else
        {
            DBG("\tPump init failed!");
        }

        // Last preparations
        LL_mDelay(1000);
        cmd::report_ready();
        adc::increment_and_sync();

        while (CDC_IsConnected() != USBD_OK) CLI_RUN(); // Note: requires DTR (i.e. hardware handshake)
        cdc_stream.write({ 0x00 }, 1); //Dumy byte to get CDC TxState (Can_Transmit) working
        HAL_Delay(100);
        DBG("USB connected, can transmit = %u\n", (CDC_Can_Transmit() == USBD_OK) ? 1 : 0);
    }
    void main()
    {
        static uint32_t tick;
        static uint32_t last_gpio_sync = 0;
        static uint32_t last_regulator_update = 0;

        tick = HAL_GetTick();
        CLI_RUN();

        // GPIO
        if (tick - last_gpio_sync > SR_SYNC_INTERVAL)
        {
            sr_io::sync();
            last_gpio_sync = tick;
        }
        a_io::poll();

        // PC commands
        cmd::poll();

        // ADC
        adc::drdy_check();
        if (adc::status == MY_ADC_STATUS_READ_PENDING)
        {
            adc::read();
            adc::increment_and_sync();
            const adc::channel_t* conc_sense_ch = adc::get_channel(pumps::get_sensing_adc_channel());
            if (conc_sense_ch)
            {
                float sense_v = conc_sense_ch->averaging_container->get_average();
                if (isfinite(sense_v)) //False if averaging container has no points yet
                {
                    pumps::set_concentration_feedback(my_math::volts_to_volume_concentration(
                        sense_v, a_io::temperature));
                    pumps::compute_pid();
                }
            }
            dac::read_current();
        }

        // DAC
        dac::process();

        //Regulator
        if (tick - last_regulator_update > REGULATOR_UPDATE_INTERVAL)
        {
            pumps::process();
            last_regulator_update = tick;
        }

        if (dump_data)
        {
            //DBG("Dumping data...");
            //ADC
            for (size_t i = 0; i < MY_ADC_MAX_MODULES; i++)
            {
                auto& m = adc::modules[i];
                if (!m.present) continue;
                for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
                {
                    assert_param(m.channels[j].averaging_container);
                    float res = m.channels[j].averaging_container->get_average();
                    //printf("ADC ch #%02u V = %8.6f\n", i * MY_ADC_CHANNELS_PER_CHIP + j, res);
                    cmd::set_adc_voltage(i * MY_ADC_CHANNELS_PER_CHIP + j, res);
                }
            }
            //DAC
            for (size_t i = 0; i < MY_DAC_MAX_MODULES; i++)
            {
                auto& m = dac::modules[i];
                if (!m.present) continue;
                cmd::set_dac_current(i, m.current);
                cmd::set_dac_corrected_voltage(i, m.corrected_setpoint);
            }
            //AIO
            for (size_t i = 0; i < a_io::in::INPUTS_NUM; i++)
            {
                cmd::set_analog_in(i, a_io::voltages[i]);   
            }
            cmd::set_temperature(a_io::temperature);

            cmd::set_status_bit(MY_CMD_STATUS_HAVE_NEW_DATA);
            dump_data = false;
        }

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
    uint16_t Stream::write(const uint8_t *buf, uint16_t len)
    {
        while (CDC_Can_Transmit() != USBD_OK);
        if (CDC_Transmit_FS(buf, len) == USBD_OK)
        {
            return len;
        }
        puts(">> Failed to send a CDC resp");
        return 0;
    }
    uint16_t Stream::availableForWrite()
    {
        return (CDC_Can_Transmit() == USBD_OK) ? SERIAL_BUFFER_SIZE : 0;
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
    void digitalWrite(pin_t& p, uint8_t state)
    {
        if (state)
            LL_GPIO_SetOutputPin(p.port, p.mask);
        else
            LL_GPIO_ResetOutputPin(p.port, p.mask);
    }
    void pinMode(pin_t& p, uint8_t mode)
    {
        if (mode == OUTPUT)
            LL_GPIO_SetPinMode(p.port, p.mask, LL_GPIO_MODE_OUTPUT);
    }
    void pin_t::set(bool state)
    {
        if (sr)
        {
            sr_io::set_output(static_cast<sr_io::out>(mask), state);
            sr_io::sync();
        }
        else
        {
            digitalWrite(*this, state);
        }
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