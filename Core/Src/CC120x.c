#include "CC120x.h"

SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

const uint16_t timeout = 0xFFFF;

void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin) {
	spi = hspi;
	GPIOx = GPIOPort;
	GPIO_Pin = GPIOPin;
	MX_SPI3_Init();
	cs_high();
	return;
}

void cs_low(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void cs_high(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

//typedef struct {
//	HAL_SPI_StateTypeDef spi_status;
//	uint8_t *TX[]
//
//} CC120X_Status;
//todo добавить uint8_t size, потому что данные в fifo не 1
//todo структуры CC120x_READWRITE, CC120x_BURST
//todo if(BURST == 1) получать пока (возможно) MISO не 0
//todo структура FIFO access; if(FIFO_ACCESS) запись/чтение в буфер

uint8_t* CC120x_8bitAccess(RWBit rwBit, uint8_t *pData, uint8_t length) {

	uint8_t *pRxData = { 0 };

	HAL_SPI_TransmitReceive(&spi, &pData, &pRxData, 1/*data length*/, timeout);

	if (rwBit == CC120X_Read)
		HAL_SPI_Receive(&spi, &pRxData, length, timeout);

	return pRxData;
}

uint8_t* CC120x_16bitAccess(RWBit rwBit, uint8_t *Command, uint8_t *Address, uint8_t length) {
	uint8_t *pRxData = { 0 };

	HAL_SPI_TransmitReceive(&spi, &Command, &pRxData, 1, timeout);
	HAL_SPI_TransmitReceive(&spi, &Address, &pRxData, 1, timeout);

	if (rwBit == CC120X_Read)
		HAL_SPI_Receive(&spi, &pRxData, length, timeout);

	return pRxData;
}

uint8_t* CC120x_SingleRegAccess(RWBit rwBit, uint16_t pData, uint8_t length) {

	uint8_t *pRxData;
	//cast to 2 Byte
	uint8_t Command = (uint8_t) ((uint16_t) pData >> 8);
	uint8_t Address = (uint8_t) ((uint16_t) pData & 0x00FF);
	Burst burst = CC120X_SingleAccess;
	uint8_t data;
	if (Command == 0) {
		data = (uint8_t*) ((uint8_t) Address | rwBit | burst);

		pRxData = CC120x_8bitAccess(rwBit, data, length);

	} else {

		data = (uint8_t*) ((uint8_t) 0x2F | rwBit | burst);

		pRxData = CC120x_16bitAccess(rwBit, data, Address, length);

	}
	return pRxData;

}

uint8_t* CC120x_WriteStrobe(uint8_t command) {
	cs_low();
	uint8_t size = 1;
	uint8_t *ret = CC120x_SingleRegAccess(CC120X_Write, command, size);
	cs_high();
	return ret;
}
uint8_t* CC120x_WriteReg(uint16_t address, uint8_t value) {
	cs_low();
	CC120x_SingleRegAccess(CC120X_Write, address, 1);
	uint8_t *ret = CC120x_SingleRegAccess(CC120X_Write, value, 1);
	cs_high();
	return ret;
}
uint8_t* CC120x_ReadReg(uint16_t address) {
	cs_low();
	uint8_t *ret = CC120x_SingleRegAccess(CC120X_Read, address, 1);
	cs_high();
	return ret;

}

//uint8_t CC120x_ReadReg(uint16_t *address, uint8_t lenght) {
//	uint8_t *tempExt = (uint8_t*) ((uint16_t) address >> 8);
//	uint8_t *tempAddr = (uint8_t*) ((uint16_t) address & 0x00FF);
//	uint8_t *pData;
//	/* Checking if this is a FIFO access -> returns chip not ready  */
////	if ((CC120X_SINGLE_TXFIFO <= tempAddr) && (tempExt == 0))
////		return STATUS_CHIP_RDYn_BM;
//	/* Decide what register space is accessed */
//	if (!tempExt) {
//		pData = CC120x_8bitAccess(CC120x_READ, (uint8_t*) address, lenght);
//	} else if ((uint8_t) tempExt == 0x2F) {
//		uint8_t *ptrTempExt = (uint8_t*) 0xAF;
//		pData = CC120x_16bitAccess(CC120x_READ, (uint8_t*) ptrTempExt,
//				(uint8_t*) tempAddr, NULL, lenght);
//	}
//	return pData;
//}
//
//HAL_StatusTypeDef CC120x_WriteSingleReg(uint8_t *address, uint8_t *command,
//		uint8_t size) {
//	HAL_StatusTypeDef status;
//	uint8_t *pData;
//	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
//	if (HAL_SPI_Transmit(&spi, &address, 1, 0xFFFF) == HAL_OK)
//		status = HAL_SPI_Transmit(&spi, &command, size, 0xFFFF);
//	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
//	return status;
//}
//
//HAL_StatusTypeDef CC120x_Write(uint8_t *command, uint8_t size) {
//	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
//	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &command, size, timeout);
//	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
//	return status;
//}
