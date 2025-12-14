/*
 * main.h
 *
 * Created: 2025-11-11 10:03:57 AM 
 *  Author: Jiheon 
 */ 

#ifndef MAIN_H_
#define MAIN_H_


#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "lcd.h"
#include "servo.h"
#include "MPU6050.h"
#include "GP2Y0A02.h"

#define Q2deg	5


servo_t servo_status; //Current Servo motor angle status
int16_t LPF_alpha = 80, LPF_scale = 100; // LPF의 alpha: LPF_alpha / LPF_scale = 0.8
volatile MPU6050_Data_t mpu6050_val;
volatile Quarternion_t filter;
volatile uint8_t data_ready_flg = 0;
inline void delay_ms(uint16_t ms);

float roll_deg, pitch_deg, yaw_deg;
float pitch_deg_stabilize, yaw_deg_stabilize;;

ISR(TIMER1_COMPA_vect)
{
	asm("nop");
}
ISR(TIMER3_COMPA_vect)
{
	MPU6050_read(&mpu6050_val);
	float ax = (float)mpu6050_val.accelX / ACCEL_SENSITIVITY;
	float ay = (float)mpu6050_val.accelY / ACCEL_SENSITIVITY;
	float az = (float)mpu6050_val.accelZ / ACCEL_SENSITIVITY;
	
	float gx = ((float)mpu6050_val.gyroX / GYRO_SENSITIVITY) * (M_PI / 180.0f);
	float gy = ((float)mpu6050_val.gyroY / GYRO_SENSITIVITY) * (M_PI / 180.0f);
	float gz = ((float)mpu6050_val.gyroZ / GYRO_SENSITIVITY) * (M_PI / 180.0f);
	
	MahonyAHRSupdateIMU(&filter, gx,gy,gz,ax,ay,az);
	roll_deg = atan2f(2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3), 1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2)) * (180.0f / M_PI);
	pitch_deg = asinf(2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1)) * (180.0f / M_PI);
	yaw_deg = atan2f(2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2), 1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3)) * (180.0f / M_PI);
	data_ready_flg=1;
}



#endif /* MAIN_H_ */