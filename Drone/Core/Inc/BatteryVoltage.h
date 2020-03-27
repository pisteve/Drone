/*
 * BatteryVoltage.h
 *
 *  Created on: Mar 26, 2020
 *      Author: sp
 */

#ifndef INC_BATTERYVOLTAGE_H_
#define INC_BATTERYVOLTAGE_H_

#include "main.h"
#include "adc.h"


void InitBatteryADC();
void BatteryVoltage(float*);

#endif /* INC_BATTERYVOLTAGE_H_ */
