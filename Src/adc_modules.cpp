#include "adc_modules.h"
#include "ads1220.h"

#define MODULE_CS 0
#define MODULE_DRDY 1
#define REFERENCE_VOLTAGE 2.048 //V
#define FULL_SCALE 0x7FFFFF //3-byte-wide integer
#define MODULE_PINS_CS 0
#define MODULE_PINS_DRDY 1
#define MAX_INTERMODULE_DELAY 2 //mS
#define OUTPUT_STRING_FORMAT "A%.2X: %+.6f\n" //Line format: "01F: +1.000000\n" 15 bytes/module (only one channel can be read at a time)
#define OUTPUT_REPORT_FORMAT "ADC Module #%u\n" \
    "\tSPI Handle: %p\n" \
    "\tCS MUX Mask: %u\n" \
    "\tDRDY Pin: %.4lX - %.4lX\n" \
    "\tChannels (%u):\n%s"
#define OUTPUT_CHANNEL_FORMAT "\t\tmux=%X cal={%.6f,%.6f};\n" // <tab>mux=X cal={1.000000,0.000000};<nl> 33 bytes/line min
#define CS_MUX_MASK(index) ((index) << 3u) //Lower half of 8 addresses coded with PA3-PA5 (shifted accordingly)

//PRIVATE
ADS1220_regs regs_buffer = ADS1220_default_regs;

void activate_cs(adc::module_t* m)
{
    LL_GPIO_SetOutputPin(adc::cs_mux_port, m->cs_mux_mask);
    LL_GPIO_ResetOutputPin(adc::cs_pin->port, adc::cs_pin->mask); // Active-low
    //Setup time after /CS assertion is 50nS for ads1220
    //On STM32F401 at 84MHz this is approximately 5 CPU cycles (3 NOPs because 2 cycles are guaranteed)
    /*__NOP();
    __NOP();
    __NOP();*/
    //Optocoupler needs a far longer delay
    user::uDelay(25);
}

void deactivate_cs(adc::module_t* m)
{
    LL_GPIO_ResetOutputPin(adc::cs_mux_port, m->cs_mux_mask);
    LL_GPIO_SetOutputPin(adc::cs_pin->port, adc::cs_pin->mask); // Active-low
    user::uDelay(75); //Optocoupler turn OFF time is longer than turn ON time!
}

float convert(int32_t adc_reading, adc::module_t* m)
{
    float res = (adc_reading * REFERENCE_VOLTAGE) / FULL_SCALE;
    if (m->channels[m->selected_channel].invert) res = -res;
    return res * m->channels[m->selected_channel].cal_coeff + m->channels[m->selected_channel].cal_offset;
}

//PUBLIC
namespace adc
{
    //Globals
    volatile uint8_t status = MY_ADC_STATUS_INITIALIZING;
    int16_t acquisition_speed = ADS1220_DR_20SPS;
    user::pin_t drdy_pin = { nDRDY_GPIO_Port, nDRDY_Pin };
    user::pin_t enable_pin = { GPIOB, LL_GPIO_PIN_15 };
    user::pin_t* cs_pin; //main CS pin, not the MUX address pins
    GPIO_TypeDef* cs_mux_port = BOARD_ADDR0_GPIO_Port;
    module_t modules[] = 
    {
        {
            .cs_mux_mask = CS_MUX_MASK(0u),
            .drdy = &drdy_pin,
            .channels = {
                {
                    .mux_conf = ADS1220_MUX_AIN0_AIN1,
                    .cal_coeff = 1,
                    .cal_offset = -0.000000,
                    .invert = false
                },
                {
                    .mux_conf = ADS1220_MUX_AIN2_AIN3,
                    .cal_coeff = 1,
                    .cal_offset = +0.000010,
                    .invert = true
                }
            }
        }
    };

    void init(SPI_HandleTypeDef* hspi, user::pin_t* spi_cs_pin)
    {
        static_assert(array_size(modules) <= MY_ADC_MAX_MODULES, "Too many ADC modules.");
        if (!hspi) dbg_usb_prints("ADC SPI interface handle is NULL!\n");
        cs_pin = spi_cs_pin;
        LL_GPIO_SetOutputPin(cs_pin->port, cs_pin->mask); //Set /CS HIGH
        LL_GPIO_ResetOutputPin(cs_mux_port, CS_MUX_MASK(3u)); //Clear CS MUX outputs (index = 0-3)
        //Configure communication members
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            m.hspi = hspi;
            m.present = false;
            for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
            {
                m.channels[j].averaging_container = new average(MY_ADC_AVERAGING);
            }
        }
        //Enable power and transievers
        LL_GPIO_SetOutputPin(enable_pin.port, enable_pin.mask); //Set ENABLE high
    }

    void probe()
    {
        status = MY_ADC_STATUS_INITIALIZING;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.hspi) continue;
            activate_cs(&m);
            m.present = ADS1220_init(m.hspi, &regs_buffer);
            if (m.present)
                ADS1220_set_data_rate(m.hspi, acquisition_speed, &regs_buffer);
            deactivate_cs(&m);
        }
    }

    void increment_and_sync()
    {
        status = MY_ADC_STATUS_INITIALIZING;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            m.last_channel = m.selected_channel;
            m.selected_channel = (m.selected_channel + 1u) % array_size(m.channels);
            activate_cs(&m);
            ADS1220_select_mux_config(m.hspi, m.channels[m.selected_channel].mux_conf, &regs_buffer);
            ADS1220_start_conversion(m.hspi);
            deactivate_cs(&m);
        }
        status = MY_ADC_STATUS_WAITING;
    }

    void read()
    {
        status = MY_ADC_STATUS_READING;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            activate_cs(&m);
            int32_t res = ADS1220_read_blocking(m.hspi, m.drdy->port, m.drdy->mask, MAX_INTERMODULE_DELAY);
            deactivate_cs(&m);
            m.channels[m.selected_channel].averaging_container->enqueue(convert(res, &m));
        }
        status = MY_ADC_STATUS_WAITING;
    }

    void drdy_callback()
    {
        if (status == MY_ADC_STATUS_WAITING)
        {
            status = MY_ADC_STATUS_READ_PENDING;
        }
    }

    void drdy_check()
    {
        if (!LL_GPIO_IsInputPinSet(nDRDY_GPIO_Port, nDRDY_Pin)) drdy_callback(); //Active low
    }

    size_t dump_last_data(char* buf, size_t max_len)
    {
        size_t written = 0;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
            {
                float res = m.channels[j].averaging_container->get_average();
                if (abs(res) < 9.999990)
                {
                    int w = snprintf(buf, max_len - written, OUTPUT_STRING_FORMAT, 0x10u * i + j, res);
                    if (w > 0) //On success, increment the variables, on error overwrite the bad line
                    {
                        written += w;
                        buf += w;
                    }
                }
            }
        }
        return written;
    }

    size_t dump_module_report(char* buf, size_t max_len)
    {
        size_t written = 0;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            char cbuf[36 * MY_ADC_CHANNELS_PER_CHIP];
            char* cbuf_p = cbuf;
            for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
            {
                auto& c = m.channels[j];
                cbuf_p += snprintf(cbuf_p, sizeof(cbuf) - (cbuf_p - cbuf), OUTPUT_CHANNEL_FORMAT, 
                    c.mux_conf, c.cal_coeff, c.cal_offset);
            }
            int w = snprintf(buf, max_len - written, OUTPUT_REPORT_FORMAT, i,
                m.hspi, m.cs_mux_mask, m.drdy->port->MODER, m.drdy->mask, MY_ADC_CHANNELS_PER_CHIP, cbuf);
            if (w > 0) //On success, increment the variables, on error overwrite the bad line
            {
                written += w;
                buf += w;
            }
        }
        return written;
    }
}