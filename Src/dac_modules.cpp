#include "dac_modules.h"
#include "ad5061.h"
#include "ina219.h"
#include <math.h>

//PRIVATE

#define MY_INA219_CAL_MAGIC 33554.4 /* Divide by ohms */
#define MY_INA219_CURRENT_LSB 1.2207E-6
#define MY_DAC_FULL_SCALE 0xFF
#define MY_DAC_REFERENCE_VOLTAGE 2.5 //V
#define OUTPUT_CURRENT_FORMAT "C%.2X: %+8.5f\n"
#define OUTPUT_REPORT_FORMAT "DAC Module #%u\n" \
    "\tSPI Handle: %p\n" \
    "\tCS Pin: %.4lX - %.4lX\n" \
    "\tI2C Handle: %p\n" \
    "\tAddress: %u\n" \
    "\tCal: {%.6f,%.6f}\n" \
    "\tRShunt: %.3f\n"

//User-friendly index to I2C address lower nibble mapping. Based on DIP switch board layout.
enum : uint8_t
{
    MY_DAC_0 = 5,
    MY_DAC_1 = 6,
    MY_DAC_2 = 4,
    MY_DAC_3 = 13,
    MY_DAC_4 = 1
};

uint16_t volts_to_code(float volts)
{
    return static_cast<uint16_t>(roundf(volts * MY_DAC_FULL_SCALE / MY_DAC_REFERENCE_VOLTAGE));
}

void activate_cs(dac::module_t* m)
{
    user::pin_t* cs = m->cs;
    LL_GPIO_ResetOutputPin(cs->port, cs->mask); // Active-low

}

void deactivate_cs(dac::module_t* m)
{
    user::pin_t* cs = m->cs;
    LL_GPIO_SetOutputPin(cs->port, cs->mask); // Active-low
}

void set_module(dac::module_t* m, float volts)
{
    activate_cs(m);
    //10nS have to pass before bits can be clocked into ad5061 after /CS assertion
    //We are running at 84MHz, 1/84e6 ~ 1/100e6 = 10e-9 = 10nS, i.e. single CPU cycle delay is sufficient
    ad5061_set_code(m->hspi, volts_to_code(volts));
    m->last_setpoint = volts;
    deactivate_cs(m);
}

//PUBLIC

namespace dac
{
    //Globals
    volatile uint8_t status;
    module_t modules[] = 
    {  
        {
            .cs = new user::pin_t(nCS_GPIO_Port, 1),
            .addr = MY_DAC_0
        }
    };

    //Public methods
    void init(SPI_HandleTypeDef* spi_instance, I2C_HandleTypeDef* i2c_instance)
    {
        static_assert(array_size(modules) <= MY_DAC_MAX_MODULES, "Too many DAC modules.");
        if (!spi_instance || !i2c_instance) user_usb_prints("DAC SPI/I2C interface handle is NULL!\n");
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            m.hspi = spi_instance;
            m.hi2c = i2c_instance;
        }
    }

    void probe()
    {
        uint16_t cfg = INA219_CONFIG_BVOLTAGERANGE_16V |
	             INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_128S_69MS |
	             INA219_CONFIG_MODE_SVOLT_CONTINUOUS;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.hspi || !m.hi2c) continue;
            m.present = INA219_Init(m.hi2c, m.addr);
            if (!m.present) continue;
            INA219_setConfig(m.hi2c, m.addr, cfg);
            INA219_setCalibration(m.hi2c, m.addr, 
                static_cast<uint16_t>(roundf(MY_INA219_CAL_MAGIC / m.r_shunt)), 
                MY_INA219_CURRENT_LSB);
            ad5061_set_mode(m.hspi, AD5061_MODE_NORMAL);
        }
    }

    void read_current()
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            m.last_current = INA219_ReadCurrent(m.hi2c, m.addr);
        }
    }

    void set_all(float volts)
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            set_module(&m, volts);
        }
    }

    size_t dump_last_currents(char* buf, size_t max_len)
    {
        size_t written = 0;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            int w = snprintf(buf, max_len - written, OUTPUT_CURRENT_FORMAT, 0x10u * i, m.last_current);
            if (w > 0)
            {
                buf += w;
                written += w;
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
            int w = snprintf(buf, max_len - written, OUTPUT_REPORT_FORMAT, i,
                m.hspi, m.cs->port->MODER, m.cs->mask, m.hi2c, m.addr, m.cal_coeff, m.cal_offset, m.r_shunt);
            if (w > 0)
            {
                buf += w;
                written += w;
            }
        }
        return written;
    }
}