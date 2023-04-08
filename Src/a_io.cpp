#include "a_io.h"

#include "average.h"

_BEGIN_STD_C

uint8_t a_io_got_new_data = 0; //uint8_t access on ARM is atomic, bool isn't (byte-aligned word)

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance != ADC1) return;
    a_io_got_new_data = 1;
}

_END_STD_C

namespace a_io
{
    const in_cal_t* calibration_database;
    const in_cal_t* temp_sensor_cal;
    float voltages[in::INPUTS_NUM];
    float temperature;

    struct a_buffer_t
    {
        uint16_t vref_1;
        uint16_t ch_6;
        uint16_t ch_7;
        uint16_t vref_2;
        uint16_t ch_8;
        uint16_t ch_9;
        uint16_t temp;
        uint16_t vref_3;
    };

    a_buffer_t buffer = {};
    uint16_t* channels[in::INPUTS_NUM] = { &buffer.ch_6, &buffer.ch_7, &buffer.ch_8, &buffer.ch_9 };

    void apply_cal(in i, uint16_t v)
    {
        voltages[i] = calibration_database[i].k * v + calibration_database[i].b;
    }

    void init(ADC_HandleTypeDef* adc, const in_cal_t* cal, const in_cal_t* temp_cal)
    {
        DBG("AIO init...");
        temp_sensor_cal = temp_cal;
        calibration_database = cal;
        HAL_ADC_Start_DMA(adc, reinterpret_cast<uint32_t*>(&buffer), sizeof(buffer) / sizeof(uint16_t));
        DBG("\tAIO init OK.");
    }
    void poll()
    {
        if (!a_io_got_new_data) return;
        a_io_got_new_data = 0;

        for (size_t i = 0; i < in::INPUTS_NUM; i++)
        {
            apply_cal(static_cast<in>(i), *(channels[i]));
        }
        temperature = (buffer.temp - temp_sensor_cal->k) / temp_sensor_cal->b + 25;
    }
} // namespace a_io
