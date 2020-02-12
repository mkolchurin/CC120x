#ifndef CC120X_H
#include "stm32f4xx_hal.h"
#include "spi.h"

#define CC120x_READ 1
#define CC120x_WRITE 0

void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin);
HAL_StatusTypeDef CC120x_WriteStrobe(uint8_t command);
uint8_t* CC120x_8bitAccess(uint8_t RW, uint8_t *address, uint8_t size);
uint8_t* CC120x_16bitAccess(uint8_t RW, uint8_t *tempExt, uint8_t *tempAddr,
		uint8_t *pData, uint8_t lenght);
uint8_t CC120x_ReadReg(uint16_t *address, uint8_t lenght);
HAL_StatusTypeDef CC120x_Write(uint8_t *command, uint8_t size);

#endif
