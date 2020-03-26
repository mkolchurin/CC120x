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

SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

#define timeout 0xFFFF

HAL_StatusTypeDef spi_receive(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Receive(&spi, pData, Size, timeout));
}

HAL_StatusTypeDef spi_transmit(uint8_t *pData, uint16_t Size)
{
	return (HAL_SPI_Transmit(&spi, pData, Size, timeout));
}

HAL_StatusTypeDef spi_transmit_receive(uint8_t *pTxData, uint8_t *pRxData,
		uint16_t Size)
{
	return (HAL_SPI_TransmitReceive(&spi, pTxData, pRxData, Size,
	timeout));
}

void cc120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin)
{
	spi = hspi;
	GPIOx = GPIOPort;
	GPIO_Pin = GPIOPin;
	MX_SPI3_Init();
	cs_high();
	cc120x_WriteStrobe(SRES);
	uint8_t pData = 0;
	do
	{
		cs_low();
		spi_receive(&pData, 1);
		cs_high();
	} while ((pData & 0x80) == 0x80);

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
uint8_t cc120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length)
{

//	uint8_t receiveData[length];
	uint8_t rSatus = 0;
	spi_transmit_receive(&address, &rSatus, 1);

	if (rw == CC120x_Read)
	{

//		uint8_t rBuf1[length];
		for (uint16_t i = 0; i < length; i++)
			spi_receive(&(rxData[i]), 1);
	}
	if ((rw == CC120x_Write) && (burst == CC120x_SingleAccess))
	{
		for (uint16_t i = 0; i < length; i++)
			spi_transmit(&(txData[i]), 1);
	}
	if ((rw == CC120x_Write) && (burst == CC120x_burstAccess))
	{
		for (uint16_t i = 0; i < length; i++)
			spi_transmit(&(txData[i]), 1);
	}

//	CC120x_data.CC120x_Status = (((uint8_t) rSatus) & (uint8_t) 0b01110000);
	return (rSatus);
}

uint8_t cc120x_16bitAccess(RWBit rw, Burst burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint8_t *rxData, uint16_t length)
{

	spi_transmit(&command, 1);

	return (cc120x_8bitAccess(rw, burst, address, txData, rxData, length));
}

uint8_t cc120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length)
{

	uint8_t rData;

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
		rData = cc120x_8bitAccess(rwBit, burst, _address, txData, rxData,
				length);

	}
	else
	{
		_command = (_command | rwBit | burst);
		rData = cc120x_16bitAccess(rwBit, burst, _command, _address, txData,
				rxData, length);
	}
	cs_high();
	return (rData);

}

void cc120xSpiReadReg(uint16_t address, uint8_t *rx, uint16_t size)
{
	cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, address, 0, rx, size);
}
void cc120xSpiWriteReg(uint16_t address, uint8_t *tx, uint16_t size)
{
	cc120x_RegAccess(CC120x_Write, CC120x_burstAccess, address, tx, 0, size);
}

/*uint8_t* cc120x_TransmitData(uint8_t *txBuffer)
 {
 uint8_t length = sizeof(txBuffer) / sizeof(txBuffer[0]);

 cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, StdFIFO, 0, 0);

 cc120x_DataTypedef CC120x_data = cc120x_RegAccess(CC120x_Write,
 CC120x_burstAccess,
 StdFIFO, txBuffer, length);

 cc120x_WriteStrobe(STX);
 return (CC120x_data);
 }*/

/*
 uint8_t* cc120x_ReceiveData(void)
 {
 uint8_t length = cc120x_ReadSingleReg(FIFO_NUM_RXBYTES).CC120x_Received[0];
 return (cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, StdFIFO, NULL, length));
 }
 */

uint8_t cc120x_WriteStrobe(uint8_t command)
{
	return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, command, NULL,
	NULL, 1));
}
/*
 uint8_t * cc120x_WriteSingleReg(uint16_t address, uint8_t value)
 {
 uint8_t _value[] =
 { value };

 return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, _value,
 1));
 }
 */
void cc120x_WriteSettings(/*registerSetting_t *registerSettings*/)
{

	uint8_t settingsSize = (sizeof(preferredSettings)
			/ sizeof((preferredSettings)[0]));
	for (uint8_t i = 0; i < settingsSize; i++)
	{
		uint8_t value[] =
		{ preferredSettings[i].data };
		uint16_t address = preferredSettings[i].addr;
		cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, value, 0,
				1);
	}

}
uint8_t cc120x_ReadSettings()
{

	uint8_t settingsSize = (sizeof(preferredSettings)
			/ sizeof((preferredSettings)[0]));
	for (uint8_t i = 0; i < settingsSize; i++)
	{
		uint8_t rxvalue = 0;
		uint16_t address = preferredSettings[i].addr;
		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, 0, &rxvalue,
				1);
		uint8_t setting_value = preferredSettings[i].data;
		if (rxvalue != setting_value)
			HAL_Delay(100);

	}
	return (1);

}

uint8_t cc120x_RSSI(void)
{
	uint8_t rssi0 = 0, rssi1 = 0;
	cc120xSpiReadReg((uint16_t) RSSI0, &rssi0, 1);
	cc120xSpiReadReg((uint16_t) RSSI1, &rssi1, 1);
	float rssi = 0;
	if ((rssi0 & 0b00000001) == 1)
	{
		rssi = (rssi1 << 4) + (rssi0 & 0b01111000);
		rssi = (rssi * 0.0625) - 128;
	}
	return (rssi);
}
/// @return number of bytes in RX buffer
uint8_t cc120x_NumRxBytes(void)
{
	uint8_t rxSize = 0;
	cc120xSpiReadReg(NUM_RXBYTES, &rxSize, 1);
	return (rxSize);
}

#define RX_FIFO_ERROR       0x11
uint8_t cc120x_ReceiveData(uint8_t *pData)
{
	uint8_t rxSize = cc120x_NumRxBytes();
	uint8_t *rxBuffer =
	{ 0 };
	// Check that we have bytes in FIFO
	if (rxSize != 0)
	{

		// Read n bytes from RX FIFO
		cc120xSpiReadReg((uint16_t) 0x3F, &rxBuffer, rxSize);
		pData = rxBuffer;
		for (int i = 0; i < rxSize; i++)
			rxBuffer[i] = 0;
	}
	else
	{
		cc120x_WriteStrobe(SFRX);
		return (1);
	}
	// Set radio back in RX
	cc120x_WriteStrobe(SRX);

	return (0);
}

/// @param pTxData TX data array
/// @param size TX data size
/// @return status; 1 transmitted; 0 error;
uint8_t cc120x_TransmittData(uint8_t **pTxData, uint8_t size)
{
	uint8_t * buf = *pTxData;
	// Write packet to TX FIFO
	cc120xSpiWriteReg(0x3F, buf, size);

	// Strobe TX to send packet
	cc120x_WriteStrobe(STX);

	if (cc120x_WriteStrobe(SNOP) == 0x7F)
	{
		cc120x_WriteStrobe(SFTX);
		return (0x7F);
	}
	return (1);
}

//void cc120x_WriteSettings1(const struct registerSetting_t registerSettings1[])
//{
//	const registerSetting_t * registerSettings = *&registerSettings1;
//	uint8_t settingsSize = (sizeof(registerSettings)
//			/ sizeof((registerSettings)[0]));
//	for (uint8_t i = 0; i < settingsSize; i++)
//	{
//	uint8_t value[] =
//	{ registerSettings[i].data };
//
//	uint8_t address = registerSettings[i].addr;
//
//	cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, value, 1);
//	}
//
//}
/*
 uint8_t* cc120x_WriteBurstReg(uint16_t startAddress, uint8_t *value,
 uint16_t length)
 {
 //uint8_t length = sizeof(value) / sizeof(value[0]);
 return (cc120x_RegAccess(CC120x_Write, CC120x_burstAccess, startAddress,
 value, length));
 }

 uint8_t* cc120x_ReadSingleReg(uint16_t address)
 {
 return (cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, NULL, 1));
 }

 uint8_t* cc120x_ReadBurstReg(uint16_t address, uint16_t length)
 {
 return (cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, address, NULL,
 length));
 }

 registerSetting_t* cc120x_ReadSettings(void)
 {

 cc120x_DataTypedef data = cc120x_ReadBurstReg(0x00, 0x2FFF);
 registerSetting_t *rSettings = malloc(0xFFFF); //XXX check malloc use

 uint16_t l = sizeof(data.CC120x_Received) / sizeof(data.CC120x_Received[0]);
 for (uint16_t i = 0x00; i < l; i += 0x0001)
 {
 rSettings[i].addr = i;
 rSettings[i].data = data.CC120x_Received[i];
 }

 return (rSettings);

 }
 */

