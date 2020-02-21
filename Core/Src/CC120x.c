#include "CC120x.h"

SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin)
{
	spi = hspi;
	GPIOx = GPIOPort;
	GPIO_Pin = GPIOPin;
	MX_SPI3_Init();
	cs_high();
	return;
}

void cs_low(void)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void cs_high(void)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

/*
 *
 *
 */
CC120x_DataTypedef CC120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint16_t length)
{

	CC120x_DataTypedef CC120x_data;

	HAL_SPI_TransmitReceive(&spi, &address, &(CC120x_data.CC120x_Status), 1,
	timeout);

	if (rw == CC120x_Read)
		HAL_SPI_Receive(&spi, CC120x_data.CC120x_Received, length, timeout);
	if ((rw == CC120x_Write) && (burst == CC120x_burstAccess))
		HAL_SPI_Transmit(&spi, txData, length, timeout);
	return CC120x_data;
}

CC120x_DataTypedef CC120x_16bitAccess(RWBit rw, uint8_t burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint16_t length)
{

	HAL_SPI_Transmit(&spi, &command, 1, timeout);

	return CC120x_8bitAccess(rw, burst, address, txData, length);
}

CC120x_DataTypedef CC120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint16_t length)
{

	CC120x_DataTypedef CC120x_data;

	uint8_t _command = (uint8_t) ((uint16_t) address >> 8);
	uint8_t _address = (uint8_t) ((uint16_t) address & 0x00FF);

	cs_low();
	if (_command == 0x00)
	{
		_address = (_address | rwBit | burst);
		CC120x_data = CC120x_8bitAccess(rwBit, burst, _address, txData, length);

	}
	else
	{
		_command = (_command | rwBit | burst);
		CC120x_data = CC120x_16bitAccess(rwBit, burst, _command, _address,
				txData, length);
	}
	cs_high();
	return CC120x_data;

}

CC120x_DataTypedef CC120x_TransmitData(uint8_t *txBuffer)
{
	uint8_t length = sizeof(txBuffer) / sizeof(txBuffer[0]);

	CC120x_RegAccess(CC120x_Write, CC120x_SingleAccess, StdFIFO, NULL, NULL);

	CC120x_DataTypedef CC120x_data = CC120x_RegAccess(CC120x_Write,
			CC120x_burstAccess,
			StdFIFO, txBuffer, length);

	CC120x_WriteStrobe(STX);

	//TODO GPIO input?
	return CC120x_data;
}
CC120x_DataTypedef CC120x_ReceiveData(void)
{
	uint8_t length = CC120x_ReadSingleReg(FIFO_NUM_RXBYTES).CC120x_Received[0];
	return CC120x_RegAccess(CC120x_Read, CC120x_burstAccess, StdFIFO, NULL,
			length);
}

CC120x_DataTypedef CC120x_WriteStrobe(uint8_t command)
{
	return CC120x_RegAccess(CC120x_Write, CC120x_SingleAccess, command, NULL, 1);
}

CC120x_DataTypedef CC120x_WriteSingleReg(uint16_t address, uint8_t value)
{
	uint8_t _value[] =
	{ value };
	return CC120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, _value,
			1);
}

void CC120x_WriteSettings(registerSetting_t *registerSettings)
{
	uint8_t settingsSize = (sizeof(registerSettings)
			/ sizeof(registerSettings[0]));
	for (uint8_t i = 0; i < settingsSize; i++)
	{
		uint8_t value[] =
		{ registerSettings[i].data };
		CC120x_RegAccess(CC120x_Write, CC120x_SingleAccess,
				registerSettings[i].addr, value, 1);
	}

}

CC120x_DataTypedef CC120x_WriteBurstReg(uint16_t startAddress, uint8_t *value)
{
	uint8_t length = sizeof(value) / sizeof(value[0]);
	return CC120x_RegAccess(CC120x_Write, CC120x_SingleAccess, startAddress,
			value, length);
}

CC120x_DataTypedef CC120x_ReadSingleReg(uint16_t address)
{
	return CC120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, NULL, 1);
}

CC120x_DataTypedef CC120x_ReadBurstReg(uint16_t address, uint16_t length)
{
	return CC120x_RegAccess(CC120x_Read, CC120x_burstAccess, address, NULL,
			length);
}

registerSetting_t*
CC120x_ReadSettings(void)
{

	CC120x_DataTypedef data = CC120x_ReadBurstReg(0x00, 0x2FFF);
	registerSetting_t *rSettings;
	for (uint16_t i = 0x00; i < 0x2FFF; i += 0x0001)
	{
		rSettings[i].addr = i;
		rSettings[i].data = data.CC120x_Received[i];
	}

	return rSettings;

}
