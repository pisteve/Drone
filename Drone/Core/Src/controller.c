/*
 * pid.c
 *
 *  Created on: Mar 25, 2020
 *      Author: sp
 */

#include "controller.h"

static enum {Kp,Ki,Kd};


static int32_t throttle = 1400;																															//1135 is minimum PWM value for propeller to spin
//static int32_t motorSignal[4] = {0};

static float dt = .004;
static float gain[3][3] = {{4,0,1},{4,0,1},{0,0,0}};
static float error = 0;
static float prev_error[3] = {0};
static float integral[3] = {0};
static float derivative = 0;
static int32_t pidSignal[3] = {0};


void MotorInit() {

	htim3.Instance->CCR1 = 1000;																																//Set Capture Control Register 1 microsecond per bit
	htim3.Instance->CCR2 = 1000;
	htim3.Instance->CCR3 = 1000;
	htim3.Instance->CCR4 = 1000;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

}


void Pid(float* measurement, float* setpoint) {

	error = setpoint[x] - measurement[x];																													//Pitch Signal Calculation
	integral[x] += error*dt;
	derivative = (error - prev_error[x]) / dt;
	pidSignal[x] = gain[x][Kp]*error + gain[x][Ki]*integral[x] + gain[x][Kd]*derivative;	prev_error[x] = error;

	error = setpoint[y] - measurement[y];																													//Roll Signal Calculation
	integral[y] += error*dt;
	derivative = (error - prev_error[y]) / dt;
	pidSignal[y] = gain[y][Kp]*error + gain[y][Ki]*integral[y] + gain[y][Kd]*derivative;
	prev_error[y] = error;

	error = setpoint[z] - measurement[z];																													//Heading Signal Calculation
	integral[z] += error*dt;
	derivative = (error-prev_error[z]) / dt;
	pidSignal[z] = gain[z][Kp]*error + gain[z][Ki]*integral[z] + gain[z][Kd]*derivative;
	prev_error[z] = error;

}


void MotorCommand(float* measurement, float* setpoint, int32_t* motorSignal) {

	Pid(measurement, setpoint);
	motorSignal[0] = throttle - pidSignal[x] + pidSignal[y] - pidSignal[z]; 										//Calculate PWM esc 1 (front-right - CCW)
	motorSignal[1] = throttle + pidSignal[x] + pidSignal[y] + pidSignal[z]; 										//Calculate PWM esc 2 (rear-right - CW)
	motorSignal[2] = throttle + pidSignal[x] - pidSignal[y] - pidSignal[z]; 										//Calculate PWM esc 3 (rear-left - CCW)
	motorSignal[3] = throttle - pidSignal[x] - pidSignal[y] + pidSignal[z]; 										//Calculate PWM esc 4 (front-left - CW)

	for(int i = 0; i < 4; i++) {
		if (motorSignal[i] < throttle)
			motorSignal[i] = throttle;
		else if (motorSignal[i] > 2000)
			motorSignal[i] = 2000;
	}

	htim3.Instance->CCR1 = motorSignal[0];
	htim3.Instance->CCR2 = motorSignal[1];
	htim3.Instance->CCR3 = motorSignal[2];
	htim3.Instance->CCR4 = motorSignal[3];

}





/* PWM Motor Signal */
/*
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
uint32_t duty_cycle_throttle_high = 1000;
htim3.Instance->CCR1 = 1000;
HAL_Delay(10000);
for(int i = duty_cycle_throttle_high; i <= 1135 ;i++){
	htim3.Instance->CCR1 = i;
	HAL_Delay(1);
}
*/
