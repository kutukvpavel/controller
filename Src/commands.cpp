#include "commands.h"

#include "dac_modules.h"
#include "adc_modules.h"
#include "sr_io.h"
#include "../ModbusPort/src/ModbusSlave.h"

#include <math.h>

void clear_stream(user::Stream* stream)
{
    char c;
    while ((c = stream->read()) != '\0') if (c == '\n') break;
}

void read_float(user::Stream* stream, float* val)
{
    char buf[9];
    stream->readBytes(reinterpret_cast<uint8_t*>(buf), 8);
    sscanf(buf, "%f", val);
}

namespace cmd
{
#define STATUS_BITS_NUM (sizeof(status_t) * __CHAR_BIT__)
#define COILS_NUM (STATUS_BITS_NUM + sr_io::out::OUTPUT_NUM)
#define REGISTERS_NUM (sizeof(modbus_registers) * __CHAR_BIT__ / 16u)
#define INPUT_REGS_NUM ()

    static Modbus* modbus;

    struct modbus_coils
    {
        status_t status = 0;
    };
    struct modbus_registers
    {
        float dac_setpoint = 0.2;
        float depolarization_percent = 0;
        float depolarization_setpoint = 0;
        motor_params_t motors[MOTORS_NUM];
    };
    modbus_registers regs = {};
    modbus_coils coils = {};

    void write_status_bit(size_t address, bool bit)
    {
        if (bit) coils.status |= _BV(address);
        else coils.status &= ~_BV(address);
    }
    void write_reg(size_t address, uint16_t val)
    {
        uint16_t* p = reinterpret_cast<uint16_t*>(&regs);
        *(p + address) = val;
    }

    uint8_t write_cb(uint8_t fc, uint16_t address, uint16_t length)
    {
        switch (fc)
        {
        case FC_WRITE_COIL:
        {
            if (address >= COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            bool b = modbus->readCoilFromBuffer(0);
            if (address < STATUS_BITS_NUM)
            {
                write_status_bit(address, b);
            }
            else
            {
                address -= STATUS_BITS_NUM;
                sr_io::set_output(static_cast<sr_io::out>(address), b);
            }
            break;
        }
        case FC_WRITE_MULTIPLE_COILS:
            if ((address + length) > COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                size_t a = address + i;
                bool b = modbus->readCoilFromBuffer(i);
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
            if (address >= REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            write_reg(address, modbus->readRegisterFromBuffer(0));
            break;
        case FC_WRITE_MULTIPLE_REGISTERS:
            if ((address + length) > REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                write_reg(address + i, modbus->readRegisterFromBuffer(i));
            }
            break;
        default:
            return STATUS_ILLEGAL_FUNCTION;
        }
        return STATUS_OK;
    }
    uint8_t read_cb(uint8_t fc, uint16_t address, uint16_t length)
    {
        switch (fc)
        {
        case FC_READ_DISCRETE_INPUT:
            if (address >= sr_io::in::INPUT_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            modbus->writeDiscreteInputToBuffer(0, sr_io::get_input(static_cast<sr_io::in>(address)));
            break;
        case FC_READ_HOLDING_REGISTERS:
            if ((address + length) > REGISTERS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            modbus->writeArrayToBuffer(0, reinterpret_cast<uint16_t*>(&regs), length);
            break;
        case FC_READ_COILS:
            if ((address + length) > COILS_NUM) return STATUS_ILLEGAL_DATA_ADDRESS;
            for (size_t i = 0; i < length; i++)
            {
                size_t a = address + i;
                if (a < STATUS_BITS_NUM)
                {
                    modbus->writeCoilToBuffer(i, get_status_bit_set(a));
                    if (_BV(a) == MY_CMD_STATUS_HAVE_NEW_DATA) reset_status_bit(MY_CMD_STATUS_HAVE_NEW_DATA);
                }
                else
                {
                    a -= STATUS_BITS_NUM;
                    modbus->writeCoilToBuffer(i, sr_io::get_output(a));
                }
            }
            break;
        case FC_READ_INPUT_REGISTERS:
            //GPIO ADC
            break;
        default:
            break;
        }
        return STATUS_OK;
    }

    void init(user::Stream& stream, motor_params_t* m)
    {
        static_assert(sizeof(modbus_registers) % 2 == 0);

        if (modbus) return;

        modbus = new Modbus(stream);
        modbus->cbVector[CB_WRITE_COILS] = write_cb;
        modbus->cbVector[CB_WRITE_HOLDING_REGISTERS] = write_cb;
        modbus->cbVector[CB_READ_COILS] = read_cb;
        modbus->cbVector[CB_READ_DISCRETE_INPUTS] = read_cb;
        modbus->cbVector[CB_READ_HOLDING_REGISTERS] = read_cb;
        modbus->cbVector[CB_READ_INPUT_REGISTERS] = read_cb;
        modbus->begin(115200);
    }
    void poll()
    {
        modbus->poll();
    }

    void report_ready()
    {
        //user_usb_prints("READY...\n");
        coils.status |= MY_CMD_STATUS_READY;
    }
    void set_status_bit(status_t mask)
    {
        coils.status |= mask;
    }
    void reset_status_bit(status_t mask)
    {
        coils.status &=~ mask;
    }
    bool get_status_bit_set(status_t mask)
    {
        return coils.status & mask;
    }
    const motor_params_t* get_motor_params(size_t i)
    {
        return &(regs.motors[i]);
    }
    float get_dac_setpoint()
    {
        return regs.dac_setpoint;
    }
    float get_depolarization_percent()
    {
        return regs.depolarization_percent;
    }
    float get_depolarization_setpoint()
    {
        return regs.depolarization_setpoint;
    }

    /*size_t report_depolarization_percent(char* output_buf, size_t max_len)
    {
        return snprintf(output_buf, max_len, DEPOLARIZATION_REPORT_FORMAT, depolarization_percent);
    }*/

    /*void process(user::Stream* stream, char* output_buf, size_t max_len)
    {
        if (!stream->available()) return;
        if (!stream->new_line_reached()) return;
        char c = stream->read();
        user_usb_prints("PARSED.\n");
        switch (c)
        {
        case 'A':
            if (status & MY_CMD_ACQUIRE)
            {
                user_usb_prints("END.\n");
            }
            else
            {
                user_usb_prints("ACQ.\n");
            }
            status ^= MY_CMD_ACQUIRE;
            break;
        case 'S':
            read_float(stream, &dac_setpoint);
            dac::set_all(dac_setpoint);
            break;
        case 'P':
        {
            float temp;
            read_float(stream, &temp);
            if (temp > 1) temp = 1;
            depolarization_percent = temp;
            if (depolarization_percent < 0.01) depolarization_percent = 0;
            if (temp < 0.01) temp = 0.01; //So that the timer doesn't fire constantly
            MY_TIM_DEPOLARIZATION->ARR = static_cast<uint16_t>(roundf(temp * 30000));
            break;
        }
        case 'D': //Setting to DAC setpoint disables depolarization
            read_float(stream, &depolarization_setpoint);
            dac::set_depolarization(depolarization_setpoint);
            break;
        case 'R':
            while (CDC_Can_Transmit() != HAL_OK); //Ensure the PARSED message gets transmitted
            HAL_NVIC_SystemReset();
            break;
        case 'E':
            status |= MY_CMD_STATUS_DAC_CORRECT; //Single-shot
            break;
        case 'I':
        {
            size_t len = adc::dump_module_report(output_buf, max_len);
            cdc_transmit_blocking(reinterpret_cast<uint8_t*>(output_buf), len);
            len = dac::dump_module_report(output_buf, max_len);
            cdc_transmit_blocking(reinterpret_cast<uint8_t*>(output_buf), len);
            break;
        }
        case 'M':
            status ^= MY_CMD_STATUS_MOTOR_EN;
            sr_io::set_output(sr_io::out::MOTOR_EN, status & MY_CMD_STATUS_MOTOR_EN);
            break;
        default:
            break;
        }
        clear_stream(stream);
    }*/
}