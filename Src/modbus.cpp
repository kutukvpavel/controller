#include "modbus.h"

Modbus* slave = NULL;

void initialize(int addr)
{
    if (slave != NULL) delete slave;
    slave = new Modbus(addr);
    slave->begin(115200);
    slave->cbVector[CB_READ_INPUT_REGISTERS] = read_input;
}

uint8_t read_input(uint8_t fc, uint16_t address, uint16_t length)
{

}

uint8_t write_output(uint8_t fc, uint16_t address, uint16_t length)
{
    
}