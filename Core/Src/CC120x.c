#include "types.h"

#include "CC120x.h"

SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

#define timeout 0xFFFF

void cs_low(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void cs_high(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef spi_receive(uint8_t *pData, uint16_t Size) {
//	while (HAL_SPI_STATE_READY != HAL_SPI_GetState(&spi)) {
//
//	}
	return (HAL_SPI_Receive(&spi, pData, Size, timeout));
}

HAL_StatusTypeDef spi_transmit(uint8_t *pData, uint16_t Size) {
//	while (HAL_SPI_STATE_READY != HAL_SPI_GetState(&spi)) {
//
//		}
	return (HAL_SPI_Transmit(&spi, pData, Size, timeout));
}

HAL_StatusTypeDef spi_transmit_receive(uint8_t *pTxData, uint8_t *pRxData,
		uint16_t Size) {
//	while (HAL_SPI_STATE_READY != HAL_SPI_GetState(&spi)) {
//
//		}
	return (HAL_SPI_TransmitReceive(&spi, pTxData, pRxData, Size,
	timeout));
}

chip_status_t cc120x_WriteStrobe(uint8_t command) {
	return (cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, command, NULL,
	NULL, 1));
}

chip_status_t cc120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin) {
	spi = hspi;
	GPIOx = GPIOPort;
	GPIO_Pin = GPIOPin;
	MX_SPI3_Init();
	cs_high();
	cc120x_WriteStrobe(SRES);
	uint8_t pData = 0;
	do {
		cs_low();
		spi_receive(&pData, 1);
		cs_high();
	} while ((pData & 0x80) == 0x80);

	return cc120x_WriteStrobe(SNOP);
}

chip_status_t cc120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length) {

	chip_status_t status;
	status.chip_status = 255;
	status.spi_status = 255;
	uint8_t chip_status_byte;
	chip_status_byte = 0;

	spi_transmit_receive(&address, &chip_status_byte, 1);

	if (rw == CC120x_Read) {
		for (uint16_t i = 0; i < length; i++)
		{
			status.spi_status = spi_receive(&(rxData[i]),
					1);
		}
	}
	if ((rw == CC120x_Write) && (burst == CC120x_SingleAccess)) {
		for (uint16_t i = 0; i < length; i++)
			status.spi_status = spi_transmit_receive(&(txData[i]),
					 &chip_status_byte, 1);
	}
	if ((rw == CC120x_Write) && (burst == CC120x_burstAccess)) {
		for (uint16_t i = 0; i < length; i++)
			status.spi_status = spi_transmit_receive(&(txData[i]),
					 &chip_status_byte, 1);
	}

	uint8_t state = (((uint8_t) chip_status_byte) & (uint8_t) 0b01110000)
			>> 4;
	switch (state) {
	case (IDLE_STATE):
		status.chip_status = IDLE_STATE;
		break;
	case (RX_STATE):
		status.chip_status = RX_STATE;
		break;
	case (TX_STATE):
		status.chip_status = TX_STATE;
		break;
	case (FSTXON_STATE):
		status.chip_status = FSTXON_STATE;
		break;
	case (CALIBRATE_STATE):
		status.chip_status = CALIBRATE_STATE;
		break;
	case (SETTLING_STATE):
		status.chip_status = SETTLING_STATE;
		break;
	case (RX_FIFO_ERROR_STATE):
		status.chip_status = RX_FIFO_ERROR_STATE;
		break;
	case (TX_FIFO_ERROR_STATE):
		status.chip_status = TX_FIFO_ERROR_STATE;
		break;
		//TODO default state?
	}
	return status;
}

chip_status_t cc120x_16bitAccess(RWBit rw, Burst burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint8_t *rxData, uint16_t length) {
	spi_transmit(&command, 1);
	return (cc120x_8bitAccess(rw, burst, address, txData, rxData, length));
}

chip_status_t cc120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length) {
	chip_status_t status;
	uint8_t _command = (uint8_t) ((uint16_t) address >> 8);
	uint8_t _address = (uint8_t) ((uint16_t) address & 0x00FF);

	cs_low();
	if (_command == 0x00) {
		_address = (_address | rwBit | burst);
		status = cc120x_8bitAccess(rwBit, burst, _address, txData, rxData,
				length);

	} else {
		_command = (_command | rwBit | burst);
		status = cc120x_16bitAccess(rwBit, burst, _command, _address, txData,
				rxData, length);
	}
	cs_high();
	return (status);
}

chip_status_t cc120x_ReadBurstRegisters(uint16_t address, uint8_t *rx,
		uint16_t size) {
	return cc120x_RegAccess(CC120x_Read, CC120x_burstAccess, address, 0, rx,
			size);
}
chip_status_t cc120x_WriteBurstRegisters(uint16_t address, uint8_t *tx,
		uint16_t size) {
	return cc120x_RegAccess(CC120x_Write, CC120x_burstAccess, address, tx, 0,
			size);
}
chip_status_t cc120x_WriteSingleRegister(uint16_t address, uint8_t value) {
	uint8_t valarr[1];
	valarr[0] = value;
	return cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, valarr,
			NULL, 1);
}
chip_status_t cc120x_ReadSingleRegister(uint16_t address, uint8_t *pValue) {
	return cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, NULL,
			pValue, 1);
}

uint8_t cc120x_WriteSettings(const registerSetting_t *registerSettings,
		uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		uint8_t value[] = { registerSettings[i].data };
		uint16_t address = registerSettings[i].addr;
		cc120x_RegAccess(CC120x_Write, CC120x_SingleAccess, address, value, 0,
				1);
	}
	HAL_Delay(100);
	return cc120x_ReadAndCompareSettings(registerSettings, size);
}
uint8_t cc120x_ReadAndCompareSettings(const registerSetting_t *registerSettings,
		uint8_t size) {
	for (uint8_t i = 0; i < size; i++) {
		uint8_t rxvalue = 0;
		uint16_t address = preferredSettings[i].addr;
		cc120x_RegAccess(CC120x_Read, CC120x_SingleAccess, address, 0,
				 &rxvalue, 1);
		uint8_t setting_value = preferredSettings[i].data;
		if (rxvalue != setting_value)
			HAL_Delay(1);
//			return (0);
	}
	return (1);
}

chip_status_t cc120x_beginReceive(void) {
	chip_status_t status;
	cc120x_WriteStrobe(SFRX);
	cc120x_WriteStrobe(SIDLE);
	while (status.chip_status != RX_STATE) {
		status = cc120x_WriteStrobe(SRX);
	}

	return status;
}

uint8_t cc120x_RSSI(void) {
	uint8_t rssi0;
	uint8_t rssi1;
	cc120x_ReadBurstRegisters((uint16_t) RSSI0, &rssi0, 1);
	cc120x_ReadBurstRegisters((uint16_t) RSSI1, &rssi1, 1);
	float rssi = 0;
	if ((rssi0 & 0b00000001) == 1) {
		rssi = (rssi1 << 4) + (rssi0 & 0b01111000);
		rssi = (rssi * 0.0625) /*- 128*/;
	}
	return (rssi);
}
/// @return number of bytes in RX buffer
uint8_t cc120x_NumRxBytes(void) {
	uint8_t rxSize;
	cc120x_ReadBurstRegisters(NUM_RXBYTES, &rxSize, 1);
	return (rxSize);
}

chip_status_t cc120x_ReceiveData(uint8_t *pData, uint8_t size) {

	chip_status_t status;
//	uint8_t rxSize = cc120x_NumRxBytes();
//	uint8_t *rxBuffer =
//	{ 0 };
	// Check that we have bytes in FIFO
	if (size != 0) {

		// Read n bytes from RX FIFO
		status = cc120x_ReadBurstRegisters((uint16_t) 0x3F, pData, size);
		//pData = rxBuffer;
//		for (int i = 0; i < rxSize; i++)
//			rxBuffer[i] = 0;
	} else {
		status = cc120x_WriteStrobe(SFRX);
		return (status);
	}
	// Set radio back in RX
	cc120x_WriteStrobe(SRX);

	return (status);
}

/// @param pTxData TX data array
/// @param size TX data size
/// @return status; 1 transmitted; 0 error;
chip_status_t cc120x_TransmittData(uint8_t *pTxData, uint8_t size) {
	uint8_t txSize;
	do {
		cc120x_ReadBurstRegisters(NUM_TXBYTES, &txSize, 1);
		if (txSize != 0)
			cc120x_WriteStrobe(STX);

	} while (txSize != 0);
//	cc120x_WriteStrobe(SNOP);
//	HAL_Delay(2);
//	uint8_t *txData = *pTxData;
	// Write packet to TX FIFO
	cc120x_WriteBurstRegisters(0x3F, pTxData, size);

	// Strobe TX to send packet
	cc120x_WriteStrobe(STX);

	return (cc120x_WriteStrobe(SNOP));
}

