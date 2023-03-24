#pragma once

#include "main.h"
#include "usbd_cdc_if.h"
#include "../cli/inc/sys_command_line.h"

#define DEBUG_USB_SERIAL 0
#define DEBUG_STEP_BY_STEP 0

//Shared API
//#define user_usb_prints(str) { uint8_t _buf[] = (str); cdc_transmit_blocking(_buf, sizeof(_buf)); }
#if DEBUG_USB_SERIAL
    #define dbg_usb_prints(str) user_usb_prints(str)
#else
    #define dbg_usb_prints(str)
#endif

#define _BV(i) (1u << (i))
#define MY_TIM_MICROS TIM5
#define MY_TIM_DEPOLARIZATION TIM4
#define SERIAL_BUFFER_SIZE APP_TX_DATA_SIZE
#define LOW 0
#define HIGH 1
#define OUTPUT 1

#define MY_STATUS_DUMP_DATA _BV(0)
#define MY_STATUS_CORRECT_DAC _BV(1)
#define MY_STATUS_DEPOLARIZE _BV(2)
#define MY_STATUS_ACQUIRE _BV(3)
#define MY_STATUS_

template<typename T, size_t s> constexpr size_t array_size(const T(&arr)[s]);
template<typename T, size_t s> constexpr size_t array_size(const T(&arr)[s]) { return s; }

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

namespace user
{
    //Globals
    extern volatile uint8_t status;

    //MAIN
    void setup(SPI_HandleTypeDef* adc_spi, SPI_HandleTypeDef* dac_spi, I2C_HandleTypeDef* dac_i2c, ADC_HandleTypeDef *adc,
        UART_HandleTypeDef *console_uart);
    void main();

    //API
    struct pin_t
    {
        pin_t(GPIO_TypeDef* p, uint32_t m) : port(p), mask(m) {}
        GPIO_TypeDef* port;
        uint32_t mask;
    };

    class Stream
    {
        uint16_t _head;
        uint16_t _tail;
        uint8_t _buffer[SERIAL_BUFFER_SIZE * 2];
        bool _newline_present;
    
    public:
        void receive(uint8_t* buf, uint16_t len);
        bool available();
        bool new_line_reached();
        uint8_t read();
        uint16_t readBytes(uint8_t* buf, uint16_t len);
        uint16_t write(const uint8_t* buf, uint16_t len);
        uint16_t availableForWrite();
    };

    uint32_t micros();
    void uDelay(uint32_t us);
    void digitalWrite(pin_t p, uint8_t state);
    void pinMode(pin_t p, uint8_t mode);
    int min(int x, int y);
    uint16_t word(uint8_t h, uint8_t l);
}