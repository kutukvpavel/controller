#include "nvs.h"

#define MY_EEPROM_ADDR 0xA0
#define MY_NVS_I2C_ADDR(mem_addr) (MY_EEPROM_ADDR | ((mem_addr & 0x700) >> 7))
#define MY_NVS_VER_ADDR 0
#define MY_NVS_START_ADDRESS 0x10
#define MY_NVS_VERSION 2u
#define MY_NVS_PAGE_SIZE 16u

namespace nvs
{
    struct PACKED_FOR_MODBUS storage_t
    {
        PACKED_FOR_MODBUS motor_params_t motors[MOTORS_NUM];
        PACKED_FOR_MODBUS a_io::in_cal_t temp_sensor_cal;
        PACKED_FOR_MODBUS a_io::in_cal_t a_in_cal[a_io::in::INPUTS_NUM];
        PACKED_FOR_MODBUS adc::ch_cal_t adc_cal[MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP];
        PACKED_FOR_MODBUS dac::cal_t dac_cal[MY_DAC_MAX_MODULES];
        PACKED_FOR_MODBUS pumps::params_t regulator_params;
    };
    PACKED_FOR_MODBUS storage_t storage =
    {
        .motors =
        {
            {
                .rate_to_speed = 1,
                .microsteps = 4,
                .teeth = 200,
                .invert_enable = false,
                .invert_error = false,
                .direction = false
            },
            {
                .rate_to_speed = 1,
                .microsteps = 4,
                .teeth = 200,
                .invert_enable = false,
                .invert_error = false,
                .direction = false
            },
            {
                .rate_to_speed = 1,
                .microsteps = 4,
                .teeth = 200,
                .invert_enable = false,
                .invert_error = false,
                .direction = false
            }
        },
        .temp_sensor_cal = { .k = 0.0025 /*V/K*/, .b = -0.76 /*V @ 25C*/ },
        .a_in_cal =
        {
            { .k = 1, .b = 0 },
            { .k = 1, .b = 0 },
            { .k = 1, .b = 0 },
            { .k = 1, .b = 0 }
        },
        .adc_cal = 
        {
            { .k = 1, .b = 0, .invert = false },
            { .k = 1, .b = 0, .invert = false },

            { .k = 1, .b = 0, .invert = false },
            { .k = 1, .b = 0, .invert = false },

            { .k = 1, .b = 0, .invert = false },
            { .k = 1, .b = 0, .invert = false },

            { .k = 1, .b = 0, .invert = false },
            { .k = 1, .b = 0, .invert = false }
        },
        .dac_cal = 
        {
            { .k = 1, .b = 0, .current_k = 1, .current_b = 0 },
            { .k = 1, .b = 0, .current_k = 1, .current_b = 0 },
            { .k = 1, .b = 0, .current_k = 1, .current_b = 0 },
            { .k = 1, .b = 0, .current_k = 1, .current_b = 0 }
        },
        .regulator_params = {
            .kP = 0.001,
            .kI = 0,
            .kD = 0,
            .low_concentration_motor_index = 1,
            .high_concentration_motor_index = 2,
            .sensing_adc_channel_index = 0,
            .low_concentration_dac_channel_index = 0,
            .high_concentration_dac_channel_index = 1,
            .total_flowrate = 100 //mL/min?
        }
    };
    
    I2C_HandleTypeDef* i2c = NULL;
    uint8_t nvs_ver = 0;
    HAL_StatusTypeDef eeprom_read(uint16_t addr, uint8_t* buf, uint16_t len)
    {
        return HAL_I2C_Mem_Read(i2c, MY_NVS_I2C_ADDR(addr), addr & 0xFF, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
    }
    HAL_StatusTypeDef eeprom_write(uint16_t addr, uint8_t* buf, uint16_t len)
    {
        assert_param(addr % MY_NVS_PAGE_SIZE == 0);
        HAL_StatusTypeDef status = HAL_OK;

        size_t full_pages = len / MY_NVS_PAGE_SIZE;
        size_t remainder = len % MY_NVS_PAGE_SIZE;
        DBG("Writing NVS: Full pages = %u, Remainder = %u", full_pages, remainder);
        uint16_t current_page_addr = addr;
        for (size_t i = 0; i < full_pages; i++)
        {
            status = HAL_I2C_Mem_Write(i2c, MY_NVS_I2C_ADDR(current_page_addr), current_page_addr & 0xFF, I2C_MEMADD_SIZE_8BIT, 
                buf, MY_NVS_PAGE_SIZE, 1000);
            if (status != HAL_OK) break;
            buf += MY_NVS_PAGE_SIZE;
            current_page_addr += MY_NVS_PAGE_SIZE;
            HAL_Delay(5);
        }
        if (status != HAL_OK) return status;
        if (remainder > 0)
        {
            status = HAL_I2C_Mem_Write(i2c, MY_NVS_I2C_ADDR(current_page_addr), current_page_addr & 0xFF, I2C_MEMADD_SIZE_8BIT, 
                buf, remainder, 1000);
        }
        return status;
    }

    PACKED_FOR_MODBUS motor_params_t* get_motor_params(size_t i)
    {
        assert_param(i < MOTORS_NUM);
        return &(storage.motors[i]);
    }
    PACKED_FOR_MODBUS a_io::in_cal_t* get_analog_input_cal(size_t i)
    {
        assert_param(i < a_io::in::INPUTS_NUM);
        return &(storage.a_in_cal[i]);
    }
    PACKED_FOR_MODBUS a_io::in_cal_t* get_temp_sensor_cal()
    {
        return &storage.temp_sensor_cal;
    }
    PACKED_FOR_MODBUS adc::ch_cal_t* get_adc_channel_cal(size_t i)
    {
        assert_param(i < array_size(storage.adc_cal));
        return &(storage.adc_cal[i]);
    }
    PACKED_FOR_MODBUS dac::cal_t* get_dac_cal(size_t i)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        return &(storage.dac_cal[i]);
    }
    PACKED_FOR_MODBUS pumps::params_t* get_regulator_params()
    {
        return &(storage.regulator_params);
    }

    HAL_StatusTypeDef init(I2C_HandleTypeDef* hi2c)
    {
        static_assert(MY_NVS_START_ADDRESS % MY_NVS_PAGE_SIZE == 0);
        HAL_StatusTypeDef ret;
        DBG("NVS init...");

        i2c = hi2c;
        if ((ret = eeprom_read(MY_NVS_VER_ADDR, &nvs_ver, sizeof(nvs_ver))) != HAL_OK) return ret;
        DBG("Detected NVS ver: %u", nvs_ver);
        if (nvs_ver != MY_NVS_VERSION) return HAL_ERROR;
        return HAL_OK;
    }
    HAL_StatusTypeDef load()
    {
        HAL_StatusTypeDef ret;

        if (!i2c) return HAL_BUSY;
        if (nvs_ver != MY_NVS_VERSION) return HAL_ERROR;
        if ((ret = eeprom_read(MY_NVS_START_ADDRESS, reinterpret_cast<uint8_t*>(&storage), sizeof(storage))) != HAL_OK) return ret;
        return HAL_OK;
    }
    HAL_StatusTypeDef save()
    {
        HAL_StatusTypeDef ret;

        if (!i2c) return HAL_BUSY;
        if ((ret = eeprom_write(MY_NVS_START_ADDRESS, reinterpret_cast<uint8_t*>(&storage), sizeof(storage))) != HAL_OK) return ret;
        DBG("NVS Data written OK. Writing NVS version...");
        uint8_t ver[] = { MY_NVS_VERSION };
        return eeprom_write(MY_NVS_VER_ADDR, ver, 1);
    }
    HAL_StatusTypeDef reset()
    {
        uint8_t zero[] = { 0 };
        return eeprom_write(MY_NVS_VER_ADDR, zero, 1);
    }
    void dump_hex()
    {
        for (size_t i = 0; i < sizeof(storage); i++)
        {
            printf("0x%02X\n", reinterpret_cast<uint8_t*>(&storage)[i]);
        }
    }
    HAL_StatusTypeDef test()
    {
        uint8_t* sequential_buffer = new uint8_t[sizeof(storage_t)];
        assert_param(sequential_buffer);

        for (size_t i = 0; i < sizeof(storage_t); i++)
        {
            sequential_buffer[i] = static_cast<uint8_t>(i);
        }
        eeprom_write(MY_NVS_START_ADDRESS, sequential_buffer, sizeof(storage_t));
        HAL_Delay(5);

        uint8_t* readback_buffer = new uint8_t[sizeof(storage_t)];
        assert_param(readback_buffer);
        eeprom_read(MY_NVS_START_ADDRESS, readback_buffer, sizeof(storage_t));
        HAL_StatusTypeDef res = (memcmp(sequential_buffer, readback_buffer, sizeof(storage_t)) == 0) ? HAL_OK : HAL_ERROR;

        delete[] sequential_buffer;
        HAL_Delay(5);
        save();
        return res;
    }
} // namespace nvs
