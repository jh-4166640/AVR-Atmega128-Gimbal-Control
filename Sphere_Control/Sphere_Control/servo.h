/*
 * servo.h
 *
 * Created: 2025-11-18 오전 11:15:03
 *  Author: jiheon choi
 */ 


#ifndef SERVO_H_
#define SERVO_H_

#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>

#define T_tick			0.5425
#define	OCR_MIN			922		// 0 deg 922
#define	OCR_MAX			4608	// 180 deg 4608
#define OCR_90			2765

/* Servo motor and gy25 conversion ratio */
/* Servo motor : GY25 = 2:3 */
// 60 : 90
#define SERVO2GY25_RATIO	(2.0f/3.0f)
#define SERVO_MAX 150
#define SERVO_MIN 30
#define SERVO_RANGE		(SERVO_MAX-SERVO_MIN)

#define YAW_MIN			-10.0f
#define YAW_MAX			10.0f
#define	DELTA_YAW		(YAW_MAX-YAW_MIN)
#define ResolutionStep	45

typedef struct
{
	uint16_t yaw;
	uint16_t pitch;
}servo_t;

static inline void delay_ms(uint16_t ms);
void Servo_Init(void);
void Timer1_16bit_FastPWM_Init(void);
static inline uint16_t DEG2OCR(uint16_t deg);
static inline void set_YawServo(uint16_t ocr);
static inline void set_PitchServo(uint16_t ocr);
static inline void set_Servo_Relative(servo_t *cur, int16_t p_add, int16_t y_add);
static inline void set_Servo_Absolute(servo_t *cur, int16_t p_deg, int16_t y_deg);


//void Servo_sample_code(void);

// 0 deg  OCR1 = 922
// 45 deg OCR1 = 1843
// 90 deg OCR1 = 2765
// 135 deg OCR1 = 3686
// 180 deg OCR1 = 4608 약간 더 도는 감이 없지 않아 있음 Calibration 필요
static inline void delay_ms(uint16_t ms)
{
	while(ms--)
	{
		_delay_ms(1);
	}
}
void Servo_Init(void)
{
	// Yaw Servo PB5 (OCR1A)
	// Pitch Servo PB6 (OCR1B)
	DDRB = 0xFF;
	DDRD = 0x00;
	Timer1_16bit_FastPWM_Init();
}
void Timer1_16bit_FastPWM_Init(void)
{
	/* Servo1 COM1A, Servo2 COM1B */
	// TOP = (fclk / (Prescaler * fpwm)) - 1
	// prescaler = 8
	// TOP = (14745600Hz / (8 * 50Hz) - 1 = 36863
	// T = (8 * (36863+1)) / 14745600 = 0.02 = 20ms
	// Ttick = 8/14745600 = 0.5425
	TCCR1A = 0x00;
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11);
	// TOP을 ICRn으로 제어
	ICR1 = 36863;
}

static inline uint16_t DEG2OCR(uint16_t deg)
{
	/* Degree를 OCR 값으로 변환 */
	// ocr = OCR_MIN + ((deg/180) * (OCR_MAX-OCR_MIN))
	float conv = (float)deg / 180.0;
	uint16_t ocr = 0;
	ocr = ceil(OCR_MIN + (conv * (OCR_MAX-OCR_MIN)));
	return ocr;
}
static inline void set_YawServo(uint16_t ocr)
{
	OCR1A = ocr;
}
static inline void set_PitchServo(uint16_t ocr)
{
	OCR1B = ocr;
}
static inline void set_Servo_Relative(servo_t *cur, int16_t p_add, int16_t y_add)
{
	// pitch_add, yaw_add
	float f_padd, f_yadd;
	f_padd = (float)p_add * SERVO2GY25_RATIO;
	f_yadd = (float)y_add * SERVO2GY25_RATIO;
	cur->pitch = (cur->pitch * 9 + (cur->pitch + (int16_t)f_padd)) / 10;
	cur->yaw = (cur->yaw * 9 + (cur->yaw + (int16_t)f_yadd)) / 10;
	
	//cur->pitch_deg += (int16_t)f_padd;
	//cur->yaw_deg += (int16_t)f_yadd;
	if(cur->pitch <= SERVO_MIN)			cur->pitch = SERVO_MIN;
	else if(cur->pitch >= SERVO_MAX)	cur->pitch = SERVO_MAX;
	if(cur->yaw <= SERVO_MIN)			cur->yaw = SERVO_MIN;
	else if(cur->yaw >= SERVO_MAX)		cur->yaw = SERVO_MAX;
	
	set_PitchServo(DEG2OCR(cur->pitch));	
	set_YawServo(DEG2OCR(cur->yaw));
}

static inline void set_Servo_Absolute(servo_t *cur, int16_t p_deg, int16_t y_deg)
{
	/*
	#define YAW_MIN			-10
	#define YAW_MAX			10
	#define	DELTA_YAW		YAW_MAX-YAW_MIN
	#define ResolutionStep	45
	*/
	float yaw_index_f;
	int16_t yaw_index;
	float yaw_mappeed_angle;
	
	uint8_t msg[16];
	
	if(y_deg > YAW_MAX) y_deg =YAW_MAX;
	else if(y_deg < YAW_MIN) y_deg = YAW_MIN;	
	yaw_index_f = (((float)y_deg - YAW_MIN) / DELTA_YAW) * (float)(ResolutionStep-1.0f);	
	yaw_index = (int16_t)roundf(yaw_index_f);
	yaw_mappeed_angle = SERVO_MIN + (float)yaw_index * (SERVO_RANGE/(float)(ResolutionStep - 1));

	cur->yaw = (uint16_t)yaw_mappeed_angle;
	set_YawServo(DEG2OCR(cur->yaw));
	
	
}
#endif /* SERVO_H_ */

