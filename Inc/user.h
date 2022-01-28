#pragma once

#include "main.h"

//Shared API
#define user_prints(str) { uint8_t _buf[] = str; CDC_Transmit_FS(_buf, sizeof(_buf)); }
#define _BV(i) (1u << i)

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
}