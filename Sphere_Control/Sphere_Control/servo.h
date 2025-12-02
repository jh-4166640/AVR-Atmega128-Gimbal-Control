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

static inline void delay_ms(uint16_t ms);
void Servo_Init(void);
void Timer1_16bit_FastPWM_Init(void);
static inline uint16_t DEG2OCR(uint16_t deg);
void Servo_sample_code(void);

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

void Servo_sample_code(void)
{
	//OCR1A = DEG2OCR(180);
	OCR1A = DEG2OCR(90);
	OCR1B = DEG2OCR(90);
	uint16_t cur_deg = 90;
	uint16_t cur_deg2 = 90;
	uint8_t lcd_deg_buffer[4];
	uint8_t lcd_deg_buffer2[4];
	uint8_t SET_DISP[] = "degree1 :";
	uint8_t SET_DISP2[] = "degree2 :";
	itoa(cur_deg, lcd_deg_buffer, 10);
	itoa(cur_deg2, lcd_deg_buffer2, 10);
	
	while(1)
	{
		uint8_t sw = (PIND & 0xff);
		uint8_t state = 0xff;
		if(sw == 0xfe || sw == 0xfd || sw == 0xfb || sw == 0xf7 || sw == 0xef || sw == 0xdf || sw == 0xbf || sw == 0x7f)
		{
			state = sw;
			delay_ms(100);
		}
		switch(state)
		{
			case 0xfe:
				delay_ms(100);
				OCR1A = DEG2OCR(0);
				OCR1B = DEG2OCR(180);
				cur_deg = 0;
				cur_deg2 = 180;
				itoa(cur_deg, lcd_deg_buffer, 10);
				itoa(cur_deg2, lcd_deg_buffer2, 10);
				break;
			case 0xfd:
				delay_ms(100);
				cur_deg+=10;
				OCR1A = DEG2OCR(cur_deg);
				itoa(cur_deg, lcd_deg_buffer, 10);
				break;
			case 0xfb:
				delay_ms(100);
				cur_deg-=10;
				OCR1A = DEG2OCR(cur_deg);
				itoa(cur_deg, lcd_deg_buffer, 10);
				break;
			case 0xf7:
				delay_ms(100);
				OCR1A = DEG2OCR(90);
				cur_deg = 90;
				itoa(cur_deg, lcd_deg_buffer, 10);
				break;
			case 0xef:
				delay_ms(100);
				OCR1A = DEG2OCR(120);
				cur_deg = 120;
				itoa(cur_deg, lcd_deg_buffer, 10);
				break;
			case 0xdf:
				
				break;
			case 0xbf:
				delay_ms(100);
				cur_deg2+=10;
				OCR1B = DEG2OCR(cur_deg2);
				itoa(cur_deg2, lcd_deg_buffer2, 10);
				break;
			case 0x7f:
				delay_ms(100);
				cur_deg2-=10;
				OCR1B = DEG2OCR(cur_deg2);
				itoa(cur_deg2, lcd_deg_buffer2, 10);
				break;
			default:
			
			break;
		}
		LCD_Clear();
		LCD_Pos(0,0);
		LCD_Str(SET_DISP);
		LCD_Pos(0,10);
		LCD_Str(lcd_deg_buffer);
		LCD_Pos(1,0);
		LCD_Str(SET_DISP2);
		LCD_Pos(1,10);
		LCD_Str(lcd_deg_buffer2);
		delay_ms(100);	
	}
}



#endif /* SERVO_H_ */

