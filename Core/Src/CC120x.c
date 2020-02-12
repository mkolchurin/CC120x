#include "CC120x.h"

SPI_HandleTypeDef spi;
GPIO_TypeDef *GPIOx;
uint16_t GPIO_Pin;

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
	uint16_t timeout = 0xFFFF;
	return CC120x_Write(command, size, timeout);
}

uint8_t CC120x_ReadSingleReg(uint8_t *address, uint8_t size, uint8_t timeout) {
	uint8_t *pRxData;
	uint8_t *pTxData = *address | 0x80;
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &pTxData, size, timeout);
	if (status == HAL_OK)
		HAL_SPI_Receive(&spi, pRxData, size, timeout);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
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

HAL_StatusTypeDef CC120x_Write(uint8_t *command, uint8_t size, uint8_t timeout) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(&spi, &command, size, timeout);
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	return status;
}
