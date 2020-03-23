/*
 * sensors.c
 *
 *  Created on: Feb 29, 2020
 *      Author: Steven Pill
 */


#include "sensors.h"
#include "spi.h"
#include "i2c.h"
#include "math.h"
#include <stdlib.h>
#include "MadgwickAHRS.h"

enum{x,y,z};
static uint8_t buffer[6] = {0};
static float average[3] = {0};																														//Average Gyro Drift

static uint8_t gyroXYZregAutoRead = 0x28 | 0xC0;																					// Top two MSB bits = 1 to Read on SPI Bus and Auto Increment
static uint8_t accXYZregAutoRead = 0x28 | 0x80;																						//Top MSB bit to Auto Increment on I2C Bus
static uint8_t magXYZregAutoRead = 0x03 | 0x80;

static float gyroRateXYZ[3] = {0};
static int16_t accXYZ[3] = {0};
static int16_t magXYZ[3] = {0};


void IMU_Init() {

  LSM303DLHC_Init();
	L3GD20_Init();
	L3GD20_Calibrate();
}


void MadgwickAHRS(float* angle) {

	L3GD20_XYZrates(gyroRateXYZ);
	LSM303DLHC_AccReadXYZ(accXYZ);

	MadgwickAHRSupdateIMU(gyroRateXYZ[x],gyroRateXYZ[y],gyroRateXYZ[z],accXYZ[x],accXYZ[y],accXYZ[z]);			//Output is global quanternion variables

  angle[x] = 2*(q0*q2 - q3*q1);																																						//Convert quanternion to euler angles

  if (abs(angle[x]) >= 1)
  		angle[x]  = copysign(M_PI/2,angle[x])*57.2958; 																											//Convert radians to degrees and use 90 degrees if out of range
  else
  		angle[x] = asin(angle[x])*57.2958;

	angle[y] = atan2((2*(q0*q1 + q2*q3)), (1-2*(q1*q1 + q2*q2)))*57.2958-1;																	//Subtract -1 from roll for accelerometer calibration

	angle[z] = 0;
}


void L3GD20_Init() {

	uint8_t Data[2] = {0};
	Data[0]= 0x20;
	Data[1]= 0x9f;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);															//Chip Select - Pulls line to 0
	HAL_SPI_Transmit(&hspi1, &Data[0], 1, 50);																				//Enable Normal Mode, XYZ rate values, 380 Hz ODR and 25Hz Cut-off
	HAL_SPI_Transmit(&hspi1, &Data[1], 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);																//Chip Select - Release line back to 1
	Data[0]= 0x23;
	Data[1]= 0x90;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &Data[0], 1, 50);
	HAL_SPI_Transmit(&hspi1, &Data[1], 1, 50);																				//Enable 500dps
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
	Data[0]= 0x24;
	Data[1]= 0x10;
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &Data[0], 1, 50);
	HAL_SPI_Transmit(&hspi1, &Data[1], 1, 50);																				//Enable High Pass Filter
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

}


void L3GD20_XYZrates (float *pfData) {

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &gyroXYZregAutoRead, 1, 50);
  HAL_SPI_Receive(&hspi1, buffer, 6, 50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  for(int i=0; i<3; i++) {																																					//Convert sensor output to radians per second angular rate
  	pfData[i]=(float)((int16_t)(((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]) * 0.00030543261);			//0.00030543261 = .01750 / 57.29578 = sensorGain / radian2deg
  }
  pfData[x] -= average[x];
  pfData[y] -= average[y];
  pfData[z] -= average[z];

}


void L3GD20_Calibrate() {

	  float RawData[3] = {0} , temp[3] = {0};
    for(int i = 0; i<2000; i++){
    	L3GD20_XYZrates(RawData);
      temp[x] += RawData[x];
      temp[y] += RawData[y];
      temp[z] += RawData[z];
      HAL_Delay(3);
    }
    average[x] = temp[x]/2000;
    average[y] = temp[y]/2000;
    average[z] = temp[z]/2000;

}


void LSM303DLHC_Init() {

	uint8_t Data[2] = {0};
	Data[0]= 0x20;
	Data[1]= 0x77;
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);																				//Accelerometer control Register 1 and Data
	Data[0]= 0x21;
	Data[1]= 0x90;
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);
	Data[0]= 0x23;
	Data[1]= 0x28;
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);

	Data[0]= 0x00;
	Data[1]= 0x18;
	HAL_I2C_Master_Transmit(&hi2c1,0x1E<<1,Data,2,50);																				//Magnetometer control Register 1 and Data
	Data[0]= 0x01;
	Data[1]= 0x20;																																						// +/-1.3 Gauss
	HAL_I2C_Master_Transmit(&hi2c1,0x1E<<1,Data,2,50);
	Data[0]= 0x02;
	Data[1]= 0x00;
	HAL_I2C_Master_Transmit(&hi2c1,0x1E<<1,Data,2,50);

}


void LSM303DLHC_AccReadXYZ(int16_t* pData) {

	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,&accXYZregAutoRead ,1,50);
	HAL_I2C_Master_Receive(&hi2c1,0x19<<1, buffer,6,50);
  for(int i=0; i<3; i++) {
    pData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]) * 4;
  }

}


void LSM303DLHC_MagReadXYZ(int16_t* pData) {

	HAL_I2C_Master_Transmit(&hi2c1,0x1E<<1,&magXYZregAutoRead ,1,50);
	HAL_I2C_Master_Receive(&hi2c1,0x1E<<1, buffer,6,50);
  pData[0]=(int16_t)(((uint16_t)buffer[0] << 8) + buffer[1]);
  pData[1]=(int16_t)(((uint16_t)buffer[4] << 8) + buffer[5]);									//Y and Z axis components address are backwards (see datasheet)
  pData[2]=(int16_t)(((uint16_t)buffer[2] << 8) + buffer[3]);

}

