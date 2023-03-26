#include "nvs.h"

#define MY_EEPROM_ADDR 0xA0
#define MY_NVS_I2C_ADDR(mem_addr) (MY_EEPROM_ADDR | ((mem_addr & 0x700) >> 7))
#define MY_NVS_VER_ADDR 0
#define MY_NVS_START_ADDRESS (MY_NVS_VER_ADDR + 1)
#define MY_NVS_VERSION 1u

namespace nvs
{
    struct storage_t
    {
        motor_params_t motors[MOTORS_NUM];
        a_io::in_cal_t temp_sensor_cal;
        a_io::in_cal_t a_in_cal[a_io::in::INPUTS_NUM];
        adc::ch_cal_t adc_cal[MY_ADC_MAX_MODULES * MY_ADC_CHANNELS_PER_CHIP];
        dac::cal_t dac_cal[MY_DAC_MAX_MODULES];
    };
    storage_t storage =
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
        return HAL_I2C_Mem_Write(i2c, MY_NVS_I2C_ADDR(addr), addr & 0xFF, I2C_MEMADD_SIZE_8BIT, buf, len, 1000);
    }

    motor_params_t* get_motor_params(size_t i)
    {
        assert_param(i < MOTORS_NUM);
        return &(storage.motors[i]);
    }
    a_io::in_cal_t* get_analog_input_cal(size_t i)
    {
        assert_param(i < a_io::in::INPUTS_NUM);
        return &(storage.a_in_cal[i]);
    }
    a_io::in_cal_t* get_temp_sensor_cal()
    {
        return &storage.temp_sensor_cal;
    }
    adc::ch_cal_t* get_adc_channel_cal(size_t i)
    {
        assert_param(i < array_size(storage.adc_cal));
        return &(storage.adc_cal[i]);
    }
    dac::cal_t* get_dac_cal(size_t i)
    {
        assert_param(i < MY_DAC_MAX_MODULES);
        return &(storage.dac_cal[i]);
    }

    HAL_StatusTypeDef init(I2C_HandleTypeDef* hi2c)
    {
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
} // namespace nvs
