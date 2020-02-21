/*
 * СС120X.cpp
 *
 *  Created on: 22 февр. 2020 г.
 *      Author: maxim
 */

#include "СС120X.h"

СС120X::СС120X() {
	// TODO Auto-generated constructor stub

}

СС120X::~СС120X() {
	// TODO Auto-generated destructor stub
}

void CsLow(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}
void CsHigh(void) {
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}
