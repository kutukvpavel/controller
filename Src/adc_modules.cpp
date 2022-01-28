#include "adc_modules.h"

#define MODULE_CS 0
#define MODULE_DRDY 1
#define REFERENCE_VOLTAGE 2.048 //V
#define FULL_SCALE 0x7FFFFF //3-byte-wide integer

//PRIVATE
user::pin_t module_pins[MY_ADC_MAX_MODULES][2] =
{
    {{nCS_GPIO_Port, 1}, {nDRDY_GPIO_Port, nDRDY_Pin}},
    {{nCS_GPIO_Port, 1}, {nDRDY_GPIO_Port, nDRDY_Pin}},
    {{nCS_GPIO_Port, 1}, {nDRDY_GPIO_Port, nDRDY_Pin}},
    {{nCS_GPIO_Port, 1}, {nDRDY_GPIO_Port, nDRDY_Pin}}
};// /CS, /DRDY
int16_t module_channels[MY_ADC_CHANNELS_PER_CHIP] = {MUX_AIN0_AIN1, MUX_AIN2_AIN3}; //Internal indexes
void* modules[MY_ADC_MAX_MODULES];

void channel_acquisition(float buffer[array_size(module_channels)][array_size(modules)], uint8_t i);

//PUBLIC
namespace adc
{
    //Globals
    volatile uint8_t status = MY_ADC_STATUS_INITIALIZING;
    float calibration_coefficients[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP] = { {1, 1} }; // V/V
    float calibration_offset[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP] = { {0.000010, 0.000010} }; //V
    int16_t channel_gain[MY_ADC_MAX_MODULES][MY_ADC_CHANNELS_PER_CHIP] = { {PGA_GAIN_1, PGA_GAIN_1} }; //Internal indexes
    int16_t acquisition_speed = DR_20SPS;
    uint16_t acquisition_period = 250; //mS = 4 Hz
    bool module_present[MY_ADC_MAX_MODULES];

    void probe()
    {
        
    }

    void initialize()
    {

        status = MY_ADC_STATUS_WAITING;
    }

    void read()
    {
        status = MY_ADC_STATUS_READING;

        status = MY_ADC_STATUS_WAITING;
    }
}

//PRIVATE
