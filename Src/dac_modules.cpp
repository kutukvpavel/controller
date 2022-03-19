#include "dac_modules.h"
#include "ad5061.h"
#include "ina219.h"
#include <math.h>

//PRIVATE

#define MY_INA219_CAL_MAGIC 33554.4 /* Divide by ohms */
#define MY_INA219_CURRENT_LSB 1.2207E-6
#define OUTPUT_DATA_FORMAT "C%.2X: %+8.5f\n" \
    "D%.2X: %+8.6f\n"
#define OUTPUT_REPORT_FORMAT "DAC Module #%u\n" \
    "\tSPI Handle: %p\n" \
    "\tCS MUX Mask: %u\n" \
    "\tI2C Handle: %p\n" \
    "\tAddress: %u\n" \
    "\tCal: {%.6f,%.6f}\n" \
    "\tRShunt: %.3f\n" \
    "\tINA Cal: %.5f\n"
#define CS_MUX_MASK(index) (((index) + 4u) << 3u) //Higher half of 8 addresses coded with PA3-PA5 (shifted accordingly)
#define MY_INA219_CURRENT_THRESHOLD 0.00001

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
    return static_cast<uint16_t>(roundf(volts * AD5061_FULL_SCALE / AD5061_REFERENCE_VOLTAGE));
}

void activate_cs(dac::module_t* m)
{
    LL_GPIO_SetOutputPin(dac::cs_mux_port, m->cs_mux_mask);
    LL_GPIO_ResetOutputPin(dac::cs_pin->port, dac::cs_pin->mask); // Active-low
}

void deactivate_cs(dac::module_t* m)
{
    LL_GPIO_ResetOutputPin(dac::cs_mux_port, m->cs_mux_mask);
    LL_GPIO_SetOutputPin(dac::cs_pin->port, dac::cs_pin->mask); // Active-low
}

void set_module_internal(dac::module_t* m, float volts)
{
    activate_cs(m);
    //10nS have to pass before bits can be clocked into ad5061 after /CS assertion
    //We are running at 84MHz, 1/84e6 ~ 1/100e6 = 10e-9 = 10nS, i.e. single CPU cycle delay is sufficient
    ad5061_set_code(m->hspi, volts_to_code(volts));
    deactivate_cs(m);
}

//PUBLIC

namespace dac
{
    //Globals
    volatile uint8_t status;
    user::pin_t enable_pin = { GPIOB, LL_GPIO_PIN_15 };
    user::pin_t* cs_pin;
    GPIO_TypeDef* cs_mux_port = BOARD_ADDR0_GPIO_Port;
    module_t modules[] = 
    {  
        {
            .cs_mux_mask = CS_MUX_MASK(0u),
            .addr = MY_DAC_1,
            .depolarization_setpoint = NAN,
            .cal_coeff = 1,
            .current_cal_offset = -0.00005,
            .r_shunt = 1
        }
    };

    //Public methods
    void init(SPI_HandleTypeDef* spi_instance, user::pin_t* spi_cs_pin, I2C_HandleTypeDef* i2c_instance)
    {
        static_assert(array_size(modules) <= MY_DAC_MAX_MODULES, "Too many DAC modules.");
        if (!spi_instance || !i2c_instance || !spi_cs_pin) dbg_usb_prints("DAC SPI/I2C/CS pin interface handle is NULL!\n");
        cs_pin = spi_cs_pin;
        LL_GPIO_SetOutputPin(cs_pin->port, cs_pin->mask); //Set /CS HIGH
        LL_GPIO_ResetOutputPin(cs_mux_port, CS_MUX_MASK(3u)); //Clear CS MUX outputs (index = 0-3)
        //Configure communication members
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            m.hspi = spi_instance;
            m.hi2c = i2c_instance;
        }
        //Enable power and transievers
        LL_GPIO_SetOutputPin(enable_pin.port, enable_pin.mask); //Set ENABLE high
    }

    void probe()
    {
        uint16_t cfg = INA219_CONFIG_BVOLTAGERANGE_16V |
	             INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_32S_17MS |
	             INA219_CONFIG_MODE_SVOLT_CONTINUOUS;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.hspi || !m.hi2c) continue;
            activate_cs(&m);
            m.present = INA219_Init(m.hi2c, m.addr);
            if (m.present) 
            {
                INA219_setConfig(m.hi2c, m.addr, cfg);
                INA219_setCalibration(m.hi2c, m.addr, 
                    static_cast<uint16_t>(roundf(MY_INA219_CAL_MAGIC / m.r_shunt)), 
                    MY_INA219_CURRENT_LSB);
                ad5061_set_mode(m.hspi, AD5061_MODE_NORMAL);
            }
            deactivate_cs(&m);
        }
    }

    void read_current()
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            m.current = INA219_ReadCurrent(m.hi2c, m.addr) + m.current_cal_offset;
        }
    }

    void set_module(size_t i, float volts)
    {
        auto& m = modules[i];
        if (!m.present) return;
        set_module_internal(&m, volts);
        m.setpoint = volts;
    }

    void set_all(float volts)
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            set_module(i, volts);
        }
    }

    void correct_for_current()
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            if ((m.prev_current - m.current) < MY_INA219_CURRENT_THRESHOLD) continue;
            m.prev_current = m.current;
            m.corrected_setpoint = m.setpoint + m.current * (m.r_shunt + AD5061_INTERNAL_RESISTANCE); //V
            activate_cs(&m);
            ad5061_set_code(m.hspi, volts_to_code(m.corrected_setpoint));
            deactivate_cs(&m);
        }
    }

    void start_depolarization()
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            if (isnan(m.depolarization_setpoint)) continue;
            if (m.is_depolarizing) continue;
            set_module_internal(&m, m.depolarization_setpoint);
            m.is_depolarizing = true;
        }
    }

    void stop_depolarization()
    {
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            if (!m.is_depolarizing) continue;
            set_module_internal(&m, m.corrected_setpoint);
            m.is_depolarizing = false;
        }
    }

    size_t dump_last_data(char* buf, size_t max_len)
    {
        size_t written = 0;
        for (size_t i = 0; i < array_size(modules); i++)
        {
            auto& m = modules[i];
            if (!m.present) continue;
            uint8_t c = 0x10u * i;
            int w = snprintf(buf, max_len - written, OUTPUT_DATA_FORMAT, c, m.current, c, m.corrected_setpoint);
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
            if (!m.present) continue;
            int w = snprintf(buf, max_len - written, OUTPUT_REPORT_FORMAT, i,
                m.hspi, m.cs_mux_mask, m.hi2c, m.addr, m.cal_coeff, m.cal_offset, m.r_shunt, m.current_cal_offset);
            if (w > 0)
            {
                buf += w;
                written += w;
            }
        }
        return written;
    }
}