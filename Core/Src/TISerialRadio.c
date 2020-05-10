/*
 * TISerialRadio.c
 *
 *  Created on: 10 мая 2020 г.
 *      Author: mkolchurin
 */

#include "TISerialRadio.h"
//#include "stdint.h"
#include "stm32f4xx.h"
#include "stdbool.h"

void *RX_Port;
uint16_t RX_Pin;
void *TX_Port;
uint16_t TX_Pin;
void *CLK_Port;
uint16_t CLK_Pin;

bool TISerialRadio_readPin(void *GPIOx, uint16_t GPIO_Pin) {
	return HAL_GPIO_ReadPin((GPIO_TypeDef*) GPIOx, GPIO_Pin);
}

void TISerialRadio_writePin(void *GPIOx, uint16_t GPIO_Pin, bool PinState) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) GPIOx, GPIO_Pin,
			(GPIO_PinState) PinState);
}

uint8_t TISerialRadio_ReceiveSync(uint64_t syncword) {
	uint64_t recsyncword = 0;
	uint8_t bitpos = 0;

	while (1) {
		if (TISerialRadio_readPin(CLK_Port, CLK_Pin)) {
			if (bitpos > 63 && recsyncword == syncword) {
				bitpos = 0;
				return 1;
			}
			bool pinstate = TISerialRadio_readPin(RX_Port, RX_Pin);
			recsyncword = (recsyncword << 1) + pinstate;
			bitpos++;
		}
	}
	return 0;
}

uint8_t TISerialRadio_Receive(uint64_t syncword, uint8_t *buf, uint8_t size) {
	GPIO_PinState pinstate = HAL_GPIO_ReadPin(rxPort, rxPin);

	if (TISerialRadio_ReceiveSync(syncword)) {

		buf[valIndex] += pinstate << (7 - counter++);
		if (counter == 8) {
			counter = 0;
			valIndex++;

			if (valIndex == length) {
				syncreceive = 0;
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				valIndex = 0;
				transmitt = 1;
				cc120x_WriteStrobe(STX);
				cc120x_WriteStrobe(STX);
			}
		}
	}
}

void TISerialRadio_init(void *RXport, uint16_t RXpin, void *TXport,
		uint16_t TXpin, void *CLKport, uint16_t CLKpin) {
	RX_Port = RXport;
	RX_Pin = RXpin;
	TX_Port = TXport;
	TX_Pin = TXpin;
	CLK_Port = CLKport;
	CLK_Pin = CLKpin;

}
