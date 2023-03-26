#include "commands.h"

#include "nvs.h"
#include "../ModbusPort/src/ModbusSlave.h"
#include "modbus_database.h"

#include <math.h>

namespace cmd
{
#define STATUS_BITS_NUM (sizeof(bitfield_t) * __CHAR_BIT__)
#define COILS_NUM (STATUS_BITS_NUM + sr_io::out::OUTPUT_NUM)
#define HOLDING_REGISTERS_NUM modbus::get_modbus_size<modbus_holding_registers>()
#define INPUT_REGS_NUM modbus::get_modbus_size<modbus_input_registers>()
#define MY_ADC_MAX_CHANNELS (MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP)

    static Modbus* bus;

    struct modbus_coils
    {
        bitfield_t commands = 0;
    };
    struct modbus_holding_registers
    {
        float dac_setpoints[MY_DAC_MAX_MODULES];
        adc::ch_cal_t adc_cals[MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP];
        dac::cal_t dac_cals[MY_DAC_MAX_MODULES];
        a_io::in_cal_t analog_input_cals[a_io::in::INPUTS_NUM];
        a_io::in_cal_t temp_sensor_cal;
        motor_params_t motor_params[MOTORS_NUM];
        float depolarization_percent[MY_DAC_MAX_MODULES];
        float depolarization_setpoint[MY_DAC_MAX_MODULES];
    };
    struct modbus_input_registers
    {
        uint16_t max_motors_num = MOTORS_NUM;
        uint16_t max_adc_modules = MY_ADC_MAX_MODULES;
        uint16_t adc_channels_per_module = MY_ADC_CHANNELS_PER_CHIP;
        uint16_t present_adc_channels = 0;
        uint16_t max_dac_modules = MY_DAC_MAX_MODULES;
        uint16_t present_dac_channels = 0;
        uint16_t aio_num = a_io::INPUTS_NUM;
        float adc_voltages[MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP];
        float dac_currents[MY_DAC_MAX_MODULES];
        float dac_corrected_currents[MY_DAC_MAX_MODULES];
        float a_in[a_io::INPUTS_NUM];
        float temperature;
    };
    modbus_coils coils = {};
    modbus_holding_registers holding = {};
    modbus_input_registers input = {};

    void write_status_bit(size_t address, bool bit)
    {
        if (bit) coils.commands |= _BV(address);
        else coils.commands &= ~_BV(address);
    }
    void write_reg(size_t address, uint16_t val)
    {
        uint16_t* p = reinterpret_cast<uint16_t*>(&holding);
        *(p + address) = val;
    }

    uint8_t write_cb(uint8_t fc, uint16_t address, uint16_t length)
    {
        printf("> Modbus write: fc = %u, addr = %u, len = %u\n", fc, address, length);

        switch (fc)
        {
        case FC_WRITE_COIL:
        {
            if (address >= COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            bool b = bus->readCoilFromBuffer(0);
            if (address < STATUS_BITS_NUM)
            {
                write_status_bit(address, b);
                printf("> Status bit written: #%u = %u\n", address, b ? 1 : 0);
            }
            else
            {
                address -= STATUS_BITS_NUM;
                sr_io::set_output(static_cast<sr_io::out>(address), b);
                printf("> SR_IO bit written: #%u = %u\n", address, b ? 1 : 0);
            }
            break;
        }
        case FC_WRITE_MULTIPLE_COILS:
            if ((address + length) > COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                size_t a = address + i;
                bool b = bus->readCoilFromBuffer(i);
                if (a < STATUS_BITS_NUM)
                {
                    write_status_bit(a, b);
                }
                else
                {
                    a -= STATUS_BITS_NUM;
                    sr_io::set_output(static_cast<sr_io::out>(a), b);
                }
            }
            break;
        case FC_WRITE_REGISTER:
            if (address >= HOLDING_REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            write_reg(address, bus->readRegisterFromBuffer(0));
            break;
        case FC_WRITE_MULTIPLE_REGISTERS:
            if ((address + length) > HOLDING_REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                write_reg(address + i, bus->readRegisterFromBuffer(i));
            }
            break;
        default:
            return STATUS_ILLEGAL_FUNCTION;
        }
        return STATUS_OK;
    }
    uint8_t read_cb(uint8_t fc, uint16_t address, uint16_t length)
    {
        printf("> Modbus read: fc = %u, addr = %u, len = %u\n", fc, address, length);

        switch (fc)
        {
        case FC_READ_DISCRETE_INPUT:
            if (address >= sr_io::in::INPUT_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            bus->writeDiscreteInputToBuffer(0, sr_io::get_input(static_cast<sr_io::in>(address)));
            break;
        case FC_READ_HOLDING_REGISTERS:
            if ((address + length) > HOLDING_REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            bus->writeArrayToBuffer(0, reinterpret_cast<uint16_t*>(&holding) + address, length);
            break;
        case FC_READ_COILS:
            if ((address + length) > COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                size_t a = address + i;
                if (a < STATUS_BITS_NUM)
                {
                    bitfield_t mask = _BV(a);
                    bool ret = get_status_bit_set(mask);
                    printf("Modbus COIL READ: #%u = %u\n", a, ret ? 1 : 0);
                    bus->writeCoilToBuffer(i, ret);
                    if (mask == MY_CMD_STATUS_HAVE_NEW_DATA) reset_status_bit(MY_CMD_STATUS_HAVE_NEW_DATA);
                }
                else
                {
                    a -= STATUS_BITS_NUM;
                    bus->writeCoilToBuffer(i, sr_io::get_output(a));
                }
            }
            break;
        case FC_READ_INPUT_REGISTERS:
        {
            if ((address + length) > INPUT_REGS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            const uint16_t* p = reinterpret_cast<uint16_t*>(&input) + address;
            printf("Modbus INPUT REG READ: #%u = %u, len = %u\n", address, *p, length);
            bus->writeArrayToBuffer(0, p, length);
            break;
        }
        default:
            break;
        }
        return STATUS_OK;
    }

    void init(user::Stream& stream, I2C_HandleTypeDef* dac_i2c)
    {
        static_assert(sizeof(modbus_holding_registers) % 2 == 0);
        static_assert(sizeof(modbus_input_registers) % 2 == 0);

        DBG("Modbus Regs Init...");
        if (nvs::init(dac_i2c) == HAL_OK)
        {
            if (nvs::load() != HAL_OK)
            {
                ERR("Failed to load NVS");
                HAL_Delay(500);
                abort();
            }
        }
        else
        {
            ERR("NVS version mismatch! Defaults will be used.");
        }

        for (size_t i = 0; i < array_size(holding.motor_params); i++)
        {
            holding.motor_params[i] = *nvs::get_motor_params(i);
        }
        for (size_t i = 0; i < a_io::in::INPUTS_NUM; i++)
        {
            holding.analog_input_cals[i] = *nvs::get_analog_input_cal(i);
        }
        holding.temp_sensor_cal = *nvs::get_temp_sensor_cal();
        for (size_t i = 0; i < MY_ADC_MAX_MODULES; i++)
        {
            for (size_t j = 0; j < MY_ADC_CHANNELS_PER_CHIP; j++)
            {
                size_t ch = i * MY_ADC_CHANNELS_PER_CHIP + j;
                holding.adc_cals[ch] = *nvs::get_adc_channel_cal(ch);
            }
        }
        for (size_t i = 0; i < MY_DAC_MAX_MODULES; i++)
        {
            holding.dac_cals[i] = *nvs::get_dac_cal(i);
        }

        if (bus) return;
        DBG("Modbus service init...");

        bus = new Modbus(stream);
        bus->cbVector[CB_WRITE_COILS] = write_cb;
        bus->cbVector[CB_WRITE_HOLDING_REGISTERS] = write_cb;
        bus->cbVector[CB_READ_COILS] = read_cb;
        bus->cbVector[CB_READ_DISCRETE_INPUTS] = read_cb;
        bus->cbVector[CB_READ_HOLDING_REGISTERS] = read_cb;
        bus->cbVector[CB_READ_INPUT_REGISTERS] = read_cb;
        bus->begin(115200);
        DBG("\tModbus initialized OK.");
    }
    void poll()
    {
        uint8_t w;
        if ((w = bus->poll()))
        {
            printf("> Modbus resp sent, %u bytes written\n", w);
        }
        if (coils.commands & MY_CMD_STATUS_SAVE_EEPROM)
        {
            nvs::save();
            coils.commands &= ~MY_CMD_STATUS_SAVE_EEPROM;
        }
    }

    void report_ready()
    {
        DBG("READY...\n");
        coils.commands |= MY_CMD_STATUS_READY;
    }
    void set_status_bit(bitfield_t mask)
    {
        coils.commands |= mask;
    }
    void reset_status_bit(bitfield_t mask)
    {
        coils.commands &=~ mask;
    }
    bool get_status_bit_set(bitfield_t mask)
    {
        return coils.commands & mask;
    }
    void set_adc_channels_present(uint16_t num)
    {
        input.present_adc_channels = num;
    }
    void set_dac_channels_present(uint16_t num)
    {
        input.present_dac_channels = num;
    }
    float get_dac_setpoint(size_t i)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        return holding.dac_setpoints[i];
    }
    void set_adc_voltage(size_t i, float v)
    {
        assert_param(i < MY_ADC_MAX_CHANNELS);
        input.adc_voltages[i] = v;
    }
    void set_dac_current(size_t i, float v)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        input.dac_currents[i] = v;
    }
    void set_dac_corrected_current(size_t i, float v)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        input.dac_corrected_currents[i] = v;
    }
    void set_analog_in(size_t i, float v)
    {
        assert_param(i < a_io::in::INPUTS_NUM);
        input.a_in[i] = v;
    }

    motor_params_t* get_motor_params(size_t i)
    {
        assert_param(i < MOTORS_NUM);
        return &(holding.motor_params[i]);
    }
    a_io::in_cal_t* get_analog_input_cal(size_t i)
    {
        assert_param(i < a_io::in::INPUTS_NUM);
        return &(holding.analog_input_cals[i]);
    }
    a_io::in_cal_t* get_temp_sensor_cal()
    {
        return &holding.temp_sensor_cal;
    }
    adc::ch_cal_t* get_adc_channel_cal(size_t i)
    {
        assert_param(i < MY_ADC_MAX_CHANNELS);
        return &(holding.adc_cals[i]);
    }
    dac::cal_t* get_dac_cal(size_t i)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        return &(holding.dac_cals[i]);
    }
}