#ifndef CC120X_H
#define CC120X_H


#define StdFIFO 0x3F

#include "types.h"
#include "CC120x_register_settings.h"



typedef enum
{
	CC120x_burstAccess = 0x40, /*0b01000000*/
	CC120x_SingleAccess = 0x00
} Burst;

typedef enum
{
	CC120x_Read = 0x80, /*0b10000000*/
	CC120x_Write = 0x00
} RWBit;

//struct cc120x_DataTypedef
//{
//	uint8_t CC120x_Status;
//	uint16_t *CC120x_Received;
//};



//typedef struct cc120x_DataTypedef cc120x_DataTypedef;

//STATE BYTE (SO_Byte & 0x10000000)
#define STATE_CHIP_RDYn		0x00 // 0xxxxxxx
//(SO_Byte & 0x01110000)
#define STATE_IDLE 			0x00 // x000xxxx
#define STATE_RX 			0x10 // x001xxxx
#define STATE_TX 			0x20 // x010xxxx
#define STATE_FSTXON 		0x30 // x011xxxx
#define STATE_CALIBRATE		0x40 // x100xxxx
#define STATE_SETTLING		0x50 // x101xxxx
#define STATE_RX_FIFO_ERROR	0x60 // x110xxxx
#define STATE_TX_FIFO_ERROR	0x70 // x111xxxx

//STROBES
#define SRES	0x30
#define SFSTXON	0x31
#define SXOFF	0x32
#define SCAL	0x33
#define SRX		0x34
#define STX		0x35
#define SIDLE	0x36
#define SAFC	0x37
#define SWOR	0x38
#define SPWD	0x39
#define SFRX	0x3A
#define SFTX	0x3B
#define SWORRST	0x3c
#define SNOP	0x3d



void cs_low(void);
void cs_high(void);
void cc120x_Init(SPI_HandleTypeDef hspi, GPIO_TypeDef *GPIOPort,
		uint16_t GPIOPin);
uint8_t cc120x_8bitAccess(RWBit rw, Burst burst, uint8_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length);
uint8_t cc120x_16bitAccess(RWBit rw, Burst burst, uint8_t command,
		uint8_t address, uint8_t *txData, uint8_t *rxData, uint16_t length);
uint8_t cc120x_RegAccess(RWBit rwBit, Burst burst, uint16_t address,
		uint8_t *txData, uint8_t *rxData, uint16_t length);
uint8_t cc120x_WriteStrobe(uint8_t command);
void cc120x_WriteSettings(/*registerSetting_t *registerSettings*/);

#endif
