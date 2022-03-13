#include "adc_modules.h"
#include "ads1220.h"

#define MODULE_CS 0
#define MODULE_DRDY 1
#define REFERENCE_VOLTAGE 2.048 //V
#define FULL_SCALE 0x7FFFFF //3-byte-wide integer
#define MODULE_PINS_CS 0
#define MODULE_PINS_DRDY 1
#define MAX_INTERMODULE_DELAY 1 //mS
#define OUTPUT_STRING_FORMAT "A%.2X: %+.6f\n" //Line format: "01F: +1.000000\n" 15 bytes/module (only one channel can be read at a time)
#define OUTPUT_REPORT_FORMAT "ADC Module #%u" \
    "\tSPI Handle: %p\n" \
    "\tCS Pin: %.4lX - %.4lX\n" \
    "\tDRDY Pin: %.4lX - %.4lX\n" \
    "\tChannels (%u):\n%s"
#define OUTPUT_CHANNEL_FORMAT "\t\tmux=%X cal={%.6f,%.6f};\n" // <tab>mux=X cal={1.000000,0.000000};<nl> 33 bytes/line min

//PRIVATE
ADS1220_regs regs_buffer = ADS1220_default_regs;

void activate_cs(adc::module_t* m)
{
    user::pin_t* cs = m->cs;
    LL_GPIO_ResetOutputPin(cs->port, cs->mask); // Active-low
    //Setup time after /CS assertion is 50nS for ads1220
    //On STM32F401 at 84MHz this is approximately 5 CPU cycles
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

void deactivate_cs(adc::module_t* m)
{
    user::pin_t* cs = m->cs;
    LL_GPIO_SetOutputPin(cs->port, cs->mask); // Active-low
}

float convert(int32_t adc_reading, adc::module_t* m)
{
    float res = (adc_reading * REFERENCE_VOLTAGE) / FULL_SCALE;
    return res * m->channels[m->selected_channel].cal_coeff + m->channels[m->selected_channel].cal_offset;
}

//PUBLIC
namespace adc
{
    //Globals
    volatile uint8_t status = MY_ADC_STATUS_INITIALIZING;
    int16_t acquisition_speed = ADS1220_DR_20SPS;
    module_t modules[] = 
    {
        {
            .cs = new user::pin_t(nCS_GPIO_Port, 1),
            .drdy = new user::pin_t(nDRDY_GPIO_Port, 1),
            .channels = {
                {
                    .mux_conf = ADS1220_MUX_AIN0_AIN1
                },
                {
                    .mux_conf = ADS1220_MUX_AIN2_AIN3
                }
            }
        }
    };

    void init(SPI_HandleTypeDef* hspi)
    {
        static_assert(array_size(modules) <= MY_ADC_MAX_MODULES, "Too many ADC modules.");
        if (!hspi) user_usb_prints("ADC SPI interface handle is NULL!\n");
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            m.hspi = hspi;
            m.present = false;
        }
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
            int32_t res = ADS1220_read_blocking(m.hspi, m.drdy->port, m.drdy->mask, MAX_INTERMODULE_DELAY);
            m.channels[m.selected_channel].last_result = convert(res, &m);
        }
        status = MY_ADC_STATUS_WAITING;
    }

    void drdy_callback()
    {
        if (status == MY_ADC_STATUS_WAITING) status = MY_ADC_STATUS_READ_PENDING;
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
            float res = m.channels[m.last_channel].last_result;
            if (abs(res) < 9.999990)
            {
                int w = snprintf(buf, max_len - written, OUTPUT_STRING_FORMAT, 0x10u * i + m.last_channel, res);
                if (w > 0) //On success, increment the variables, on error overwrite the bad line
                {
                    written += w;
                    buf += w;
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
            char cbuf[36 * MY_ADC_CHANNELS_PER_CHIP];
            char* cbuf_p = cbuf;
            for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
            {
                auto& c = m.channels[j];
                cbuf_p += snprintf(cbuf_p, sizeof(cbuf) - (cbuf_p - cbuf), OUTPUT_CHANNEL_FORMAT, 
                    c.mux_conf, c.cal_coeff, c.cal_offset);
            }
            int w = snprintf(buf, max_len - written, OUTPUT_REPORT_FORMAT, i,
                m.hspi, m.cs->port->MODER, m.cs->mask, m.drdy->port->MODER, m.drdy->mask, MY_ADC_CHANNELS_PER_CHIP, cbuf);
            if (w > 0) //On success, increment the variables, on error overwrite the bad line
            {
                written += w;
                buf += w;
            }
        }
        return written;
    }
}