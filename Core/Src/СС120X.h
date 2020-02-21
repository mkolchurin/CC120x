/*
 * СС120X.h
 *
 *  Created on: 22 февр. 2020 г.
 *      Author: maxim
 */

#ifndef SRC_СС120X_H_
#define SRC_СС120X_H_

#include "spi.h"

class СС120X {
public:
	СС120X();
	virtual ~СС120X();
private:
	void CsLow(void);
	void CsHigh(void);

};

#endif /* SRC_СС120X_H_ */
