/*
 * BatteryVoltage.c
 *
 *  Created on: Mar 26, 2020
 *      Author: sp
 */

#include <BatteryVoltage.h>

static uint32_t adcVal = 0;

void InitBatteryADC() {

	HAL_ADC_Start(&hadc1);
}


void BatteryVoltage(float* voltage) {

	adcVal  = HAL_ADC_GetValue (&hadc1);

	*voltage = (float)(4*(adcVal * 3.26)/1024);
	//*voltage = (float)adcVal;
}
