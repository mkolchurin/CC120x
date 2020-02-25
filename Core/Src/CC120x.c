


/*
 * TODO chip status byte
 * (received status) & 0x01110000 (0x70) = chip status byte
 * chip status byte can be received when header byte, data byte, or command strobe is sent on the SPI interface
 *
 * ((received status) & 0x10000000 == 0) means chip is ready
 *
 *
 *
 *
 * TODO ???? When a SRES strobe is issued the CSn pin must be kept low and wait for SO to go low again before
 * the next header byte can be issued, as shown in Figure 5.
 *
 *
 *
 * TODO transmit package = (flush STX strobe, wait until status byte change to notTX state (for example, IDLE state))
 * TODO receive package = (being RX state (flush SRX strobe), wait until status byte change to notRX state (for example, IDLE state))
 */




#include "types.h"

#include "CC120x.h"
#include <CC120x_register_settings.h>


SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

void cc120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
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
cc120x_DataTypedef cc120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint16_t length)
{

	cc120x_DataTypedef CC120x_data;

	HAL_SPI_TransmitReceive(&spi, &address, &(CC120x_data.CC120x_Status), 1,
	timeout);

	if (rw == CC120x_Read)
	{
		CC120x_data.CC120x_Received = malloc(length);
		HAL_SPI_Receive(&spi, CC120x_data.CC120x_Received, length, timeout);
	}
	if ((rw == CC120x_Write) && (burst == CC120x_burstAccess))
	{
		HAL_SPI_Transmit(&spi, txData, length, timeout);
	}
	return (CC120x_data);
}

cc120x_DataTypedef cc120x_16bitAccess(RWBit rw, uint8_t burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint16_t length)
{

	HAL_SPI_Transmit(&spi, &command, 1, timeout);

	return (cc120x_8bitAccess(rw, burst, address, txData, length));
}

cc120x_DataTypedef cc120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint16_t length)
{

	cc120x_DataTypedef CC120x_data;

	uint8_t _command = (uint8_t) ((uint16_t) address >> 8);
	uint8_t _address = (uint8_t) ((uint16_t) address & 0x00FF);

	cs_low();
	/*TODO while(SO != low state)
	 * When CSn is pulled low, the MCU must wait until CC120X SO pin
	 * goes low before starting to transfer the header byte
	*/
	if (_command == 0x00)
	{
		_address = (_address | rwBit | burst);
		CC120x_data = cc120x_8bitAccess(rwBit, burst, _address, txData, length);

	}
	else
	{
		_command = (_command | rwBit | burst);
		CC120x_data = cc120x_16bitAccess(rwBit, burst, _command, _address,
				txData, length);
	}
	cs_high();
	return (CC120x_data);

}

cc120x_DataTypedef cc120x_TransmitData(uint8_t *txBuffer)
{
	uint8_t length = sizeof(txBuffer) / sizeof(txBuffer[0]);

	cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, StdFIFO, 0, 0);

	cc120x_DataTypedef CC120x_data = cc120x_RegAccess(CC120x_Write,
			CC120x_burstAccess,
			StdFIFO, txBuffer, length);

	cc120x_WriteStrobe(STX);

	/*TODO GPIO input?*/
	return (CC120x_data);
}
cc120x_DataTypedef cc120x_ReceiveData(void)
{
	uint8_t length = cc120x_ReadSingleReg(FIFO_NUM_RXBYTES).CC120x_Received[0];
	return (cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, StdFIFO, NULL,
			length));
}

cc120x_DataTypedef cc120x_WriteStrobe(uint8_t command)
{
	return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, command, NULL,
			1));
}

cc120x_DataTypedef cc120x_WriteSingleReg(uint16_t address, uint8_t value)
{
	uint8_t _value[] =
	{ value };

	return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, _value,
			1));
}

void cc120x_WriteSettings(registerSetting_t *registerSettings)
{
	uint8_t settingsSize = (sizeof(registerSettings)
			/ sizeof(registerSettings[0]));
	for (uint8_t i = 0; i < settingsSize; i++)
	{
		uint8_t value[] =
		{ registerSettings[i].data };
		cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess,
				registerSettings[i].addr, value, 1);
	}

}

cc120x_DataTypedef cc120x_WriteBurstReg(uint16_t startAddress, uint8_t *value)
{
	uint8_t length = sizeof(value) / sizeof(value[0]);
	return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, startAddress,
			value, length));
}

cc120x_DataTypedef cc120x_ReadSingleReg(uint16_t address)
{
	return (cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, NULL, 1));
}

cc120x_DataTypedef cc120x_ReadBurstReg(uint16_t address, uint16_t length)
{
	return (cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, address, NULL,
			length));
}

registerSetting_t* cc120x_ReadSettings(void)
{

	cc120x_DataTypedef data = cc120x_ReadBurstReg(0x00, 0x2FFF);
	registerSetting_t *rSettings = malloc(0xFFFF); //XXX check malloc use
	for (uint16_t i = 0x00; i < 0x2FFF; i += 0x0001)
	{
		rSettings[i].addr = i;
		rSettings[i].data = data.CC120x_Received[i];
	}

	return (rSettings);

}
