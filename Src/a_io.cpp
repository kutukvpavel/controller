#include "a_io.h"

#include "average.h"
#include <math.h>

#define PACKED_FOR_DMA __aligned(sizeof(uint32_t))

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
    PACKED_FOR_MODBUS const in_cal_t* calibration_database;
    PACKED_FOR_MODBUS const in_cal_t* temp_sensor_cal;
    float voltages[in::INPUTS_NUM];
    average temp_average(10);
    float temperature = 298; //K
    average vref_average(10);
    float vref = 3.3; //V

    struct PACKED_FOR_DMA a_buffer_t
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

    PACKED_FOR_DMA a_buffer_t buffer = { };
    uint16_t* channels[in::INPUTS_NUM] = { &buffer.ch_6, &buffer.ch_7, &buffer.ch_8, &buffer.ch_9 };

    void apply_cal(in i, uint16_t v)
    {
        voltages[i] = calibration_database[i].k * v + calibration_database[i].b;
    }

    void init(ADC_HandleTypeDef* adc, const in_cal_t* cal, const in_cal_t* temp_cal)
    {
        static_assert(sizeof(in_cal_t) % sizeof(float) == 0);

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
        float vref_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(
            static_cast<uint16_t>(roundf((buffer.vref_1 + buffer.vref_2 + buffer.vref_3) / 3.0f)), LL_ADC_RESOLUTION_12B);
        vref_average.enqueue(vref_mv / 1000.0f);
        vref = vref_average.get_average();
        float temp_c = __LL_ADC_CALC_TEMPERATURE(vref * 1000.0f, buffer.temp, LL_ADC_RESOLUTION_12B);
        temp_average.enqueue((temp_c + 273.15) * temp_sensor_cal->k + temp_sensor_cal->b);
        temperature = temp_average.get_average();
    }
} // namespace a_io
