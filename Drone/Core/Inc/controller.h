/*
 * pid.h
 *
 *  Created on: Mar 25, 2020
 *      Author: sp
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"
#include "tim.h"

void MotorInit();
void MotorCommand(float*,float*,int32_t*);
void Pid(float*,float*);

#endif /* INC_CONTROLLER_H_ */
