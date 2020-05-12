/*
 * TISerialRadio.h
 *
 *  Created on: 10 мая 2020 г.
 *      Author: maxim
 */

#ifndef INC_TISERIALRADIO_H_
#define INC_TISERIALRADIO_H_

#include "stm32f4xx.h"

uint8_t TISerialRadio_CLK();
uint8_t TISerialRadio_readPin(void *GPIOx, uint16_t GPIO_Pin);

void TISerialRadio_writePin(void *GPIOx, uint16_t GPIO_Pin, uint8_t PinState);

uint8_t TISerialRadio_ReceiveSync(uint64_t syncword, uint32_t Timeout);
uint8_t TISerialRadio_ReadByte();
uint8_t TISerialRadio_Receive(uint8_t *buf, uint8_t buf_size, uint32_t Timeout);
void TISerialRadio_init(void *RXport, uint16_t RXpin, void *TXport,
		uint16_t TXpin, void *CLKport, uint16_t CLKpin);

uint8_t TISerialRadio_Transmitt(uint8_t *buf, uint8_t buf_size,
		uint32_t Timeout);
uint8_t TISerialRadio_TransmittSync(uint64_t syncword, uint8_t *buf,
		uint8_t buf_size, uint32_t Timeout);
#endif /* INC_TISERIALRADIO_H_ */
