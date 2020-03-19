/*
 * sensors.c
 *
 *  Created on: Feb 29, 2020
 *      Author: Steven Pill
 */


#include "sensors.h"
#include "spi.h"

enum{x,y,z};
static uint8_t XYZregAutoRead = 0x28 | 0xC0;																						// Top two MSB bits = 1 to Read on SPI Bus and Auto Increment
static uint8_t gyro_out[6] = {0};
static float average[3] = {0};

/* Initialize L3GD20 Control Registers 1,4 and 5 */
void L3GD20_Init(uint8_t* L3GD20_CTRL_REG1_DATA, uint8_t* L3GD20_CTRL_REG4_DATA, uint8_t* L3GD20_CTRL_REG5_DATA) {

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);																	//Chip Select - Pulls line to 0
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG1_ADDR, 1, 50);																//Enable Normal Mode, XYZ rate values, 380 Hz ODR and 25Hz Cut-off
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG1_DATA, 1, 50);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);																		//Chip Select - Release line back to 1

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG4_ADDR, 1, 50);
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG4_DATA, 1, 50);																//Enable 500dps
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG5_ADDR, 1, 50);
	HAL_SPI_Transmit(&hspi1, L3GD20_CTRL_REG5_DATA, 1, 50);																//Enable High Pass Filter
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

}

void L3GD20_XYZrates (float *pfData) {

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &XYZregAutoRead, 1, 50);
  HAL_SPI_Receive(&hspi1, gyro_out, 6, 50);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  for(int i=0; i<3; i++) {
  	pfData[i]=(float)((int16_t)(((uint16_t)gyro_out[2*i+1] << 8) + gyro_out[2*i]) * .01750);	//Convert raw output to angular rate
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








