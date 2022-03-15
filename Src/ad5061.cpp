#include "ad5061.h"

//PRIVATE

#define AD5061_SOFTWARE_RESET 0xFFF
#define AD5061_MODE_MASK (_BV(16) | _BV(17))
#define AD5061_CODE_MASK 0xFF
#define AD5061_CONTROL_REG_WIDTH 3 //24 bits = 3 bytes

uint32_t register_template = AD5061_MODE_NORMAL;
uint8_t buffer[AD5061_CONTROL_REG_WIDTH];

void send(SPI_HandleTypeDef* hspi, uint32_t reg)
{
    register_template = reg;
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&reg);
    for (size_t i = 0; i < AD5061_CONTROL_REG_WIDTH; i++)
    {
        buffer[i] = bytes[AD5061_CONTROL_REG_WIDTH - 1 - i]; //Sadly, AD5061 accepts data in big-endian format
    }
    HAL_SPI_Transmit(hspi, buffer, AD5061_CONTROL_REG_WIDTH, 100);
}

//PUBLIC

void ad5061_set_code(SPI_HandleTypeDef* hspi, uint16_t code)
{
    send(hspi, (register_template & AD5061_MODE_MASK) | code);
}

void ad5061_set_mode(SPI_HandleTypeDef* hspi, uint32_t mode)
{
    send(hspi, (register_template & AD5061_CODE_MASK) | mode);
}

void ad5061_reset(SPI_HandleTypeDef* hspi)
{
    send(hspi, AD5061_SOFTWARE_RESET);
}