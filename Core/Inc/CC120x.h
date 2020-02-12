#ifndef CC120X_H
#include "stm32f4xx_hal.h"
#include "spi.h"


void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin);
HAL_StatusTypeDef CC120x_WriteStrobe(uint8_t command);
uint8_t CC120x_ReadSingleReg(uint8_t *address, uint8_t size, uint8_t timeout);
HAL_StatusTypeDef CC120x_WriteSingleReg(uint8_t *address, uint8_t *command,
		uint8_t size);
HAL_StatusTypeDef CC120x_Write(uint8_t *command, uint8_t size, uint8_t timeout);


#endif
