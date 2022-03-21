#include "ads1220.h"

ADS1220_regs ADS1220_default_regs = { 0x00, 0x04, 0x20, 0x00 }; //0x20 - 50Hz-only rejection

void ADS1220_writeRegister(SPI_HandleTypeDef *hspi, uint8_t address, uint8_t value)
{
	uint8_t arr[2] =
	{ static_cast<uint8_t>(ADS1220_WREG | (address << 2)), value };

	HAL_SPI_Transmit(hspi, arr, 2, 100);
}

uint8_t ADS1220_readRegister(SPI_HandleTypeDef *hspi, uint8_t address)
{
	uint8_t data[2] =
	{ 0, 0 };

	uint8_t txd[2] =
	{ static_cast<uint8_t>(ADS1220_RREG | (address << 2)), 0xFF };

	HAL_SPI_TransmitReceive(hspi, txd, data, 2, 100); // When doing bidirectional, transmit a dummy byte(0xFF), 2 in total, received register is in [1]
	return data[1];
}

void ADS1220_reset(SPI_HandleTypeDef *hspi)
{
	const uint8_t cmd = ADS1220_RESET;
	HAL_SPI_Transmit(hspi, (uint8_t*) &cmd, 1, 100);
}

/**
 * @brief Reset device, load configuration, read it back
 * 
 * @param hspi 
 * @param r 
 * @return device present
 */
bool ADS1220_init(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	ADS1220_reset(hspi);
	HAL_Delay(100);

	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS, r->cfg_reg2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS, r->cfg_reg3);

	HAL_Delay(10);

	uint8_t CR0 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS);
	uint8_t CR1 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS);
	uint8_t CR2 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS);
	uint8_t CR3 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS);

	return (CR0 == r->cfg_reg0 && CR1 == r->cfg_reg1 && CR2 == r->cfg_reg2 && CR3 == r->cfg_reg3);
}

void ADS1220_start_conversion(SPI_HandleTypeDef *hspi)
{
	const uint8_t cmd = ADS1220_START;
	HAL_SPI_Transmit(hspi, (uint8_t*) &cmd, 1, 100);
}

void ADS1220_enable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~_BV(0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_disable_PGA(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg0 |= _BV(0);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_set_conv_mode_continuous(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg1 |= _BV(2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_set_conv_mode_single_shot(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	r->cfg_reg1 &= ~_BV(2);
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_set_data_rate(SPI_HandleTypeDef *hspi, int datarate, ADS1220_regs *r)
{
	r->cfg_reg1 &= ~ADS1220_REG_CONFIG1_DR_MASK;
	r->cfg_reg1 |= datarate;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS, r->cfg_reg1);
}

void ADS1220_select_mux_config(SPI_HandleTypeDef *hspi, int channels_conf, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~ADS1220_REG_CONFIG0_MUX_MASK;
	r->cfg_reg0 |= channels_conf;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

void ADS1220_set_pga_gain(SPI_HandleTypeDef *hspi, int pgagain, ADS1220_regs *r)
{
	r->cfg_reg0 &= ~ADS1220_REG_CONFIG0_PGA_GAIN_MASK;
	r->cfg_reg0 |= pgagain;
	ADS1220_writeRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS, r->cfg_reg0);
}

uint8_t* ADS1220_get_config(SPI_HandleTypeDef *hspi, ADS1220_regs *r)
{
	static uint8_t cfgbuf[4];

	r->cfg_reg0 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG0_ADDRESS);
	r->cfg_reg1 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG1_ADDRESS);
	r->cfg_reg2 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG2_ADDRESS);
	r->cfg_reg3 = ADS1220_readRegister(hspi, ADS1220_CONFIG_REG3_ADDRESS);

	cfgbuf[0] = r->cfg_reg0;
	cfgbuf[1] = r->cfg_reg1;
	cfgbuf[2] = r->cfg_reg2;
	cfgbuf[3] = r->cfg_reg3;

	return cfgbuf;
}

int32_t ADS1220_read_blocking(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout) // Timeout should be at least as long as sampletime+some clock cycles, obviously
{
	uint8_t time = 0;

	while (HAL_GPIO_ReadPin(DRDY_PORT, DRDY_PIN) == GPIO_PIN_SET)
	{
		HAL_Delay(1); // This is a bit hacky
		time++;
		if (time >= timeout)
			return 0;
	}

    return ADS1220_read(hspi);
}

int32_t ADS1220_read(SPI_HandleTypeDef *hspi)
{
    uint8_t SPIbuf[3];
	int32_t result32 = 0;
	long int bit24;

    HAL_SPI_Receive(hspi, SPIbuf, 3, 100);

	bit24 = SPIbuf[0];
	bit24 = (bit24 << 8) | SPIbuf[1];
	bit24 = (bit24 << 8) | SPIbuf[2]; //Converting 3 bytes to a 24 bit int

	bit24 = (bit24 << 8);
	result32 = (bit24 >> 8); //Converting 24 bit two's complement to 32 bit two's complement

	return result32;
}

int32_t ADS1220_read_singleshot(SPI_HandleTypeDef *hspi, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout)
{
	ADS1220_start_conversion(hspi);

	return ADS1220_read_blocking(hspi, DRDY_PORT, DRDY_PIN, timeout);
}

int32_t ADS1220_read_singleshot_channel(SPI_HandleTypeDef *hspi, uint8_t channel_num, ADS1220_regs *r, GPIO_TypeDef *DRDY_PORT, uint16_t DRDY_PIN, uint16_t timeout)
{
	ADS1220_select_mux_config(hspi, channel_num, r);

	ADS1220_start_conversion(hspi);

	return ADS1220_read_blocking(hspi, DRDY_PORT, DRDY_PIN, timeout);
}