/*
 * sensors.h
 *
 *  Created on: Feb 29, 2020
 *      Author: sp
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"

/*IMU Functions */
void IMU_Init();
void MadgwickAHRS(float*);

void L3GD20_Init();
void L3GD20_Calibrate();
void L3GD20_XYZrates (float*);

void LSM303DLHC_Init();
void LSM303DLHC_AccReadXYZ(int16_t*);
void LSM303DLHC_MagReadXYZ(int16_t*);

#endif /* INC_SENSORS_H_ */
