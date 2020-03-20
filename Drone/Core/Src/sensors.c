/*
 * sensors.c
 *
 *  Created on: Feb 29, 2020
 *      Author: Steven Pill
 */


#include "sensors.h"
#include "spi.h"
#include "i2c.h"

enum{x,y,z};
static uint8_t gyroXYZregAutoRead = 0x28 | 0xC0;																						// Top two MSB bits = 1 to Read on SPI Bus and Auto Increment
static uint8_t gyroXYZbuffer[6] = {0};
static float average[3] = {0};
static 	uint8_t accXYZbuffer[6];
static 	uint8_t accXYZregAutoRead = 0x28 | 0x80;

/* Initialize L3GD20 Control Registers 1,4 and 5 */
void L3GD20_Init(uint8_t L3GD20_CTRL_REG1_DATA, uint8_t L3GD20_CTRL_REG4_DATA, uint8_t L3GD20_CTRL_REG5_DATA) {

	uint8_t ctrl1Reg = 0x20, ctrl4Reg = 0x23, ctrl5Reg = 0x24;

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);																	//Chip Select - Pulls line to 0
	HAL_SPI_Transmit(&hspi1, &ctrl1Reg, 1, 50);																//Enable Normal Mode, XYZ rate values, 380 Hz ODR and 25Hz Cut-off
	HAL_SPI_Transmit(&hspi1, &L3GD20_CTRL_REG1_DATA, 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);																		//Chip Select - Release line back to 1

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &ctrl4Reg, 1, 50);
	HAL_SPI_Transmit(&hspi1, &L3GD20_CTRL_REG4_DATA, 1, 50);																//Enable 500dps
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &ctrl5Reg, 1, 50);
	HAL_SPI_Transmit(&hspi1, &L3GD20_CTRL_REG5_DATA, 1, 50);																//Enable High Pass Filter
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

}

void L3GD20_XYZrates (float *pfData) {

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &gyroXYZregAutoRead, 1, 50);
  HAL_SPI_Receive(&hspi1, gyroXYZbuffer, 6, 50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  for(int i=0; i<3; i++) {
  	pfData[i]=(float)((int16_t)(((uint16_t)gyroXYZbuffer[2*i+1] << 8) + gyroXYZbuffer[2*i]) * .01750);	//Convert raw output to angular rate
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
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);																				//Control Register 1 and Data
	Data[0]= 0x21;
	Data[1]= 0x90;
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);
	Data[0]= 0x23;
	Data[1]= 0x28;
	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,Data,2,50);

}

void LSM303DLHC_AccReadXYZ(int16_t* pData) {

	HAL_I2C_Master_Transmit(&hi2c1,0x19<<1,&accXYZregAutoRead ,1,50);
	HAL_I2C_Master_Receive(&hi2c1,0x19<<1, accXYZbuffer, 6, 50);
  for(int i=0; i<3; i++) {
    pData[i]=((int16_t)((uint16_t)accXYZbuffer[2*i+1] << 8) + accXYZbuffer[2*i]) * 4;
  }

}



