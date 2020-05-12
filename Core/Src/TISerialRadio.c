/*
 * TISerialRadio.c
 *
 *  Created on: 10 мая 2020 г.
 *      Author: mkolchurin
 */

#include "TISerialRadio.h"

void *RX_Port;
uint16_t RX_Pin;
void *TX_Port;
uint16_t TX_Pin;
void *CLK_Port;
uint16_t CLK_Pin;
uint8_t CLK = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == CLK_Pin) {
		CLK = 1;
	}
}
uint8_t TISerialRadio_CLK() {
	if (CLK == 1) {
		CLK = 0;
		return 1;
	}
	return 0;
}
uint8_t TISerialRadio_readPin(void *GPIOx, uint16_t GPIO_Pin) {
	return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
}

void TISerialRadio_writePin(void *GPIOx, uint16_t GPIO_Pin, uint8_t PinState) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
}

uint8_t TISerialRadio_ReadByte() {
	uint8_t val = 0;
	for (uint8_t bitCounter = 0; bitCounter != 8;) {
		if (TISerialRadio_CLK() == 1) {
			uint8_t pinstate = TISerialRadio_readPin(RX_Port, RX_Pin);
			val = (val << 1) + pinstate;
			bitCounter++;
		}
	}
	return val;
}


uint8_t TISerialRadio_ReceiveSync(uint64_t syncword, uint32_t Timeout) {
	uint64_t recsyncword = 0;
	uint8_t bitpos = 0;
	uint32_t tickstart = HAL_GetTick();
	while ((bitpos < 63) || (recsyncword != syncword)) {
		if (CLK == 1) {
			CLK = 0;
			uint8_t pinstate = TISerialRadio_readPin(RX_Port, RX_Pin);
			recsyncword = (recsyncword << 1) + pinstate;
			bitpos++;
		}
		if ((HAL_GetTick() - tickstart) > Timeout)
			return 0;
	}
	bitpos = 0;
	return (HAL_GetTick());
}


uint8_t TISerialRadio_Receive(uint8_t *buf, uint8_t buf_size, uint32_t Timeout) {
	uint32_t tickstart = HAL_GetTick();
	for (int i = 0; i < buf_size; i++)
		buf[i] = TISerialRadio_ReadByte();
	if ((HAL_GetTick() - tickstart) > Timeout)
		return 0;
	return 1;
}

uint8_t TISerialRadio_TransmittSync(uint64_t syncword, uint8_t *buf,
		uint8_t buf_size, uint32_t Timeout) {
	uint8_t *sync = (uint8_t*) &syncword;
	TISerialRadio_Transmitt(sync, sizeof(syncword), Timeout);
	return TISerialRadio_Transmitt(buf, buf_size, Timeout);

}
uint8_t TISerialRadio_Transmitt(uint8_t *buf, uint8_t buf_size,
		uint32_t Timeout) {
	uint32_t tickstart = HAL_GetTick();
	uint8_t bitpos = 0;
	for (int i = 0; i < buf_size;) {
		if (CLK == 1) {
			CLK = 0;
			for (uint8_t bitpos = 0; bitpos != 8; bitpos++) {
				uint8_t state = ((buf[i] & (0x80 >> bitpos)) >> (7 - bitpos));
				TISerialRadio_writePin(TX_Port, TX_Pin, state);
			}
			i++;

			if ((HAL_GetTick() - tickstart) > Timeout)
				return 0;
		}
	}
	return 1;

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
