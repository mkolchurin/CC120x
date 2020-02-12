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
	return;
}
HAL_StatusTypeDef CC120x_WriteStrobe(uint8_t command) {
	uint8_t size = 1;
	return CC120x_Write(command, size);
}

uint8_t* CC120x_8bitAccess(uint8_t RW, uint8_t *address, uint8_t lenght) {
	uint8_t *pRxData;
	uint8_t *pTxData;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	uint8_t *AddressBuf = ((uint8_t) address | 0x80);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &AddressBuf, lenght,
			timeout);
	if (RW == CC120x_READ)
		HAL_SPI_Receive(&spi, &pRxData, lenght, timeout);
	else if (RW == CC120x_WRITE)
		HAL_SPI_Transmit(&spi, &pTxData, lenght, timeout);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	return pRxData;
}

uint8_t* CC120x_16bitAccess(uint8_t RW, uint8_t *tempExt, uint8_t *tempAddr,
		uint8_t *pData, uint8_t lenght) {
	uint8_t *pRxData;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&spi, &tempExt, lenght, timeout);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &tempAddr, lenght,
			timeout);
	if (RW == CC120x_READ)
		HAL_SPI_Receive(&spi, &pRxData, lenght, timeout);
	else if (RW == CC120x_WRITE)
		HAL_SPI_Transmit(&spi, &tempExt, lenght, timeout);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

uint8_t CC120x_ReadReg(uint16_t *address, uint8_t lenght) {
	uint8_t *tempExt = (uint8_t*) ((uint16_t) address >> 8);
	uint8_t *tempAddr = (uint8_t*) ((uint16_t) address & 0x00FF);
	uint8_t *pData;
	/* Checking if this is a FIFO access -> returns chip not ready  */
//	if ((CC120X_SINGLE_TXFIFO <= tempAddr) && (tempExt == 0))
//		return STATUS_CHIP_RDYn_BM;
	/* Decide what register space is accessed */
	if (!tempExt) {
		pData = CC120x_8bitAccess(CC120x_READ, (uint8_t*) address, lenght);
	} else if ((uint8_t) tempExt == 0x2F) {
		uint8_t *ptrTempExt = (uint8_t*) 0xAF;
		pData = CC120x_16bitAccess(CC120x_READ, (uint8_t*) ptrTempExt,
				(uint8_t*) tempAddr, NULL, lenght);
	}
	return pData;
}

HAL_StatusTypeDef CC120x_WriteSingleReg(uint8_t *address, uint8_t *command,
		uint8_t size) {
	HAL_StatusTypeDef status;
	uint8_t *pData;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&spi, &address, 1, 0xFFFF) == HAL_OK)
		status = HAL_SPI_Transmit(&spi, &command, size, 0xFFFF);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	return status;
}

HAL_StatusTypeDef CC120x_Write(uint8_t *command, uint8_t size) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &command, size, timeout);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	return status;
}
