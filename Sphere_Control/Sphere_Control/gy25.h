/*
 * gy25.h
 *
 * Created: 2025-11-25 오전 11:05:20
 *  Author: jiheon choi
 */ 

#ifndef GY25_H_
#define GY25_H_
#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "uart.h"
#include "lcd.h"


#define COMMAND_START				0xA5
#define QUERY_MODE					0x51
#define AUTO_DIRECT_MODE			0x52
#define AUTO_ASCII_MODE				0x53
#define CORRECTION_MODE_PITCH		0x54
#define CORRECTION_MODE_YAW			0x55

#define MSG_FIRST_CORR0				"Correction Mode"
#define MSG_FIRST_CORR1				"Don't move 4sec"
#define MSG_PITCH_CORR				"Setting level" 
#define MSG_YAW_CORR				"Setting Heading"
#define MSG_WAIT					"Wait...."
#define MSG_CALIBRATING				"Calibrating 4sec"

#define ERROR_VAL					801
#define FRAME_START		0xAA
#define FRAME_END		0x55

#define FRAME_S			0
#define YAW_H			1
#define YAW_L			2
#define PITCH_H			3
#define PITCH_L			4
#define ROLL_H			5
#define ROLL_L			6
#define FRAME_E			7

uint8_t gy25_buffer[8];
uint8_t gy25_idx;


typedef struct
{
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	
}gy25_t;


void GY25_Init(void);
gy25_t read_GY25(void);


#endif /* GY25_H_ */
