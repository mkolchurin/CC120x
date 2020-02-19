#ifndef CC120X_H
#include "stm32f4xx_hal.h"
#include "spi.h"

typedef enum {
	CC120X_burstAccess = 0x40, //0b01000000
	CC120X_SingleAccess = 0x00
} Burst;

typedef enum {
	CC120X_Read = 0x80, //0b10000000
	CC120X_Write = 0x00
} RWBit;

void cs_low(void);
void cs_high(void);
void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin);
uint8_t* CC120x_WriteStrobe(uint8_t value);
uint8_t* CC120x_8bitAccess(RWBit rwBit, uint8_t *pData, uint8_t lenght);
uint8_t* CC120x_16bitAccess(RWBit rwBit, uint8_t *Command, uint8_t *Address,
		uint8_t lenght);
uint8_t* CC120x_SingleRegAccess(RWBit rwBit, uint16_t pData, uint8_t lenght);

#endif
