#include "ina219.h"

float ina219_current_lsb;

/*
 * @brief:		Read a register from the IN219 sensor.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&hi2c, addr)
 * @param:		register address in hexadecimal
 * @retval:		16 bit unsigned integer that represents the register's contents.
 */
uint16_t Read16(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t Register)
{
	uint8_t Value[2];

	HAL_I2C_Mem_Read(hi2c, (addr << 1), Register, 1, Value, 2, 1000);

	return ((Value[0] << 8) | Value[1]);
}

/*
 * @brief:		Write to a register on the IN219 sensor.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&hi2c, addr)
 * @param:		Register address in hexadecimal
 * @param:		16 bit integer in hexadecimal that is the value you want to write to the register.
 * @retval:		HAL_StatusTypeDef, this will include an enum value representing
 * 				if the I2C transmission was successful or not.
 * 				typedef enum
				{
				  HAL_OK       = 0x00U,
				  HAL_ERROR    = 0x01U,
				  HAL_BUSY     = 0x02U,
				  HAL_TIMEOUT  = 0x03U
				} HAL_StatusTypeDef;
 */
HAL_StatusTypeDef Write16(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t Register, uint16_t Value)
{
	uint8_t buf[2];
	buf[0] = (Value >> 8) & 0xff;  // upper byte
	buf[1] = (Value >> 0) & 0xff; // lower byte
	return HAL_I2C_Mem_Write(hi2c, (addr << 1), Register, 1, buf, 2, 1000);
}

/*
 *  @brief:	  	Gets the raw current value (16-bit signed integer, so +-32767)
 *  @param:		Pointer to the device object that was made from the struct. EX:  (&hi2c, addr)
 *  @retval:	The raw current reading
 */
int16_t INA219_ReadCurrent_raw(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	int16_t result = Read16(hi2c, addr, INA219_REG_CURRENT);

	return (result );
}

/*
 * @brief:  	Gets the current value in mA, taking into account the
 *          	config settings and current LSB
 * @param:		Pointer to the device object that was made from the struct. EX:  (&hi2c, addr)
 * @return: 	The current reading convereted to amps
 */
float INA219_ReadCurrent(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	int16_t result = INA219_ReadCurrent_raw(hi2c, addr);

	return (result * ina219_current_lsb);
}

/*
 * @brief: 		This function will read the shunt voltage level.
 * @param:		Pointer to the device object that was made from the struct. EX:  (&hi2c, addr)
 * @retval:		Returns voltage level in mili-volts. This value represents the difference
 * 				between the voltage of the power supply and the bus voltage after the shunt
 * 				resistor.
 */
float INA219_ReadShuntVolage(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	int16_t result = Read16(hi2c, addr, INA219_REG_SHUNTVOLTAGE);

	return (result * 0.01); // ????
}

void INA219_Reset(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	Write16(hi2c, addr, INA219_REG_CONFIG, INA219_CONFIG_RESET);
	HAL_Delay(1);
}

void INA219_setCalibration(I2C_HandleTypeDef* hi2c, uint8_t addr, uint16_t CalibrationData, float current_lsb)
{
    ina219_current_lsb = 0;
	Write16(hi2c, addr, INA219_REG_CALIBRATION, CalibrationData);
}

uint16_t INA219_getConfig(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	uint16_t result = Read16(hi2c, addr, INA219_REG_CONFIG);
	return result;
}

void INA219_setConfig(I2C_HandleTypeDef* hi2c, uint8_t addr, uint16_t Config)
{
	Write16(hi2c, addr, INA219_REG_CONFIG, Config);
}

void INA219_setPowerMode(I2C_HandleTypeDef* hi2c, uint8_t addr, uint8_t Mode)
{
	uint16_t config = INA219_getConfig(hi2c, addr);

	switch (Mode) {
		case INA219_CONFIG_MODE_POWERDOWN:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_POWERDOWN & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(hi2c, addr, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(hi2c, addr, config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(hi2c, addr, config);
			break;

		case INA219_CONFIG_MODE_ADCOFF:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_ADCOFF & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(hi2c, addr, config);
			break;
	}
}

/**
 * @brief 
 * 
 * @param hi2c 
 * @param addr 
 * @return uint8_t (0 for success)
 */
uint8_t INA219_Init(I2C_HandleTypeDef* hi2c, uint8_t addr)
{
	uint8_t ready = HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 3, 2);

	if (ready == HAL_OK)
	{
		INA219_Reset(hi2c, addr);
		return 0;
	}
	else
	{
		return 1;
	}
}