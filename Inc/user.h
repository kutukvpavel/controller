#pragma once

#include "main.h"
#include "usbd_cdc_if.h"

//Shared API
#define user_prints(str) { uint8_t _buf[] = str; CDC_Transmit_FS(_buf, sizeof(_buf)); }
#define _BV(i) (1u << i)
#define MY_TIM_MICROS TIM5
#define SERIAL_BUFFER_SIZE APP_TX_DATA_SIZE
#define LOW 0
#define HIGH 1
#define OUTPUT 1

template<typename T, size_t s> constexpr size_t array_size(const T(&arr)[s]);
template<typename T, size_t s> constexpr size_t array_size(const T(&arr)[s]) { return s; }

namespace user
{
    //Globals

    //MAIN
    void setup();
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
    
    public:
        void receive(uint8_t* buf, uint16_t len);
        bool available();
        uint8_t read();
        uint16_t readBytes(uint8_t* buf, uint16_t len);
        uint16_t write(uint8_t* buf, uint16_t len);
        uint16_t availableForWrite();
    };

    uint32_t micros();
    void digitalWrite(pin_t p, uint8_t state);
    void pinMode(pin_t p, uint8_t mode);
    int min(int x, int y);
    uint16_t word(uint8_t h, uint8_t l);
}