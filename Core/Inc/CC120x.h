#ifndef CC120X_H

#define timeout 0xFFFF
#define StdFIFO 0x3F

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "registers.h"

typedef enum
{
	CC120x_burstAccess = 0x40, //0b01000000
	CC120x_SingleAccess = 0x00
} Burst;

typedef enum
{
	CC120x_Read = 0x80, //0b10000000
	CC120x_Write = 0x00
} RWBit;

typedef struct
{
	uint8_t CC120x_Status;
	uint8_t CC120x_Received[0xFF];
} CC120x_DataTypedef;

void cs_low(void);
void cs_high(void);
void CC120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin);
CC120x_DataTypedef CC120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint16_t length);
CC120x_DataTypedef CC120x_16bitAccess(RWBit rw, uint8_t burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint16_t length);
CC120x_DataTypedef CC120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint16_t length);
CC120x_DataTypedef CC120x_TransmitData(uint8_t *txBuffer);
CC120x_DataTypedef CC120x_WriteStrobe(uint8_t command);
CC120x_DataTypedef CC120x_WriteSingleReg(uint16_t address, uint8_t value);
void CC120x_WriteSettings(registerSetting_t *registerSettings);
CC120x_DataTypedef CC120x_WriteBurstReg(uint16_t startAddress, uint8_t *value);
CC120x_DataTypedef CC120x_ReadSingleReg(uint16_t address);
CC120x_DataTypedef CC120x_ReadBurstReg(uint16_t address, uint16_t length);
registerSetting_t* CC120x_ReadSettings(void);
CC120x_DataTypedef CC120x_ReceiveData(void);

#endif
