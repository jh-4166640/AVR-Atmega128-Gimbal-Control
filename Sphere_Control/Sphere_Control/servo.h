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
#define SERVO_MAX_YAW		150
#define SERVO_MIN_YAW		30
#define SERVO_RANGE_YAW		(SERVO_MAX_YAW-SERVO_MIN_YAW)
#define SERVO_MAX_PITCH		170
#define SERVO_MIN_PITCH		90
#define Trans_deg_yaw		30.0f
#define Trans_deg_pitch		80.0f
#define YAWRATIO			(((float)SERVO_MAX_YAW - 90.0f)/Trans_deg_yaw)
#define PITCHRATIO			(((float)SERVO_MAX_PITCH - 90.0f)/Trans_deg_pitch)


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
static inline void set_Servo_angle(servo_t *cur, float pdeg, float stabz_p, float ydeg, float stabz_y);
static inline int16_t set_ServoPitch_Relative(const uint16_t cur, float p_deg, float stabilize_val);
static inline int16_t set_ServoYaw_Relative(const uint16_t cur, float y_deg, float stabilize_val);



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
static inline void set_Servo_angle(servo_t *cur, float pdeg, float stabz_p, float ydeg, float stabz_y)
{
	const int16_t SERVO_SMOOTH_STEP = 2;
	int16_t target_p, target_y;
	int16_t cur_p = cur->pitch;
	int16_t cur_y = cur->yaw;
	target_p = set_ServoPitch_Relative(cur_p, pdeg, stabz_p);
	target_y = set_ServoYaw_Relative(cur_y, ydeg, stabz_y);
	
	
	while(target_p != cur->pitch || target_y != cur->yaw)
	{
		if(cur->pitch < target_p) {
			cur_p += SERVO_SMOOTH_STEP;
			if(cur_p > target_p) cur_p = target_p;
		}
		else if (cur_p > target_p) {
			cur_p -= SERVO_SMOOTH_STEP;
			if (cur_p < target_p) cur_p = target_p;
		}
		
		if (cur_y < target_y) {
			cur_y += SERVO_SMOOTH_STEP;
			if (cur_y > target_y) cur_y = target_y;
			} else if (cur_y > target_y) {
			cur_y -= SERVO_SMOOTH_STEP;
			if (cur_y < target_y) cur_y = target_y;
		}
		
		set_PitchServo(DEG2OCR(cur_p));
		set_YawServo(DEG2OCR(cur_y));
		
		cur->pitch = cur_p;
		cur->yaw = cur_y;
		delay_ms(1);
	}
	delay_ms(20);
}
static inline int16_t set_ServoPitch_Relative(const uint16_t cur, float p_deg, float stabilize_val)
{
	const float pass_val = 2.0f;
	float delta = p_deg - stabilize_val;
	if(fabs(delta) < pass_val) return cur;
	
	int16_t target_angle = cur + ((int16_t)roundf(delta * PITCHRATIO));
	if(target_angle <= SERVO_MIN_PITCH) target_angle = SERVO_MIN_PITCH;
	else if(target_angle >= SERVO_MAX_PITCH) target_angle = SERVO_MAX_PITCH;
	
	return target_angle;
}

static inline int16_t set_ServoYaw_Relative(const uint16_t cur, float y_deg, float stabilize_val)
{
	const float pass_val = 0.5f;
	float delta = y_deg - stabilize_val;
	if(fabs(delta) < pass_val) return cur;
	
	int16_t target_angle = cur + ((int16_t)roundf(delta * YAWRATIO));
	if(target_angle > SERVO_MAX_YAW) target_angle = SERVO_MAX_YAW;
	else if(target_angle < SERVO_MIN_YAW) target_angle = SERVO_MIN_YAW;
	
	return target_angle;
}


#endif /* SERVO_H_ */


