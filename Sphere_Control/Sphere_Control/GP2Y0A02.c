/*
 * GP2Y0A02.c
 *
 * Created: 2025-12-14 오전 12:36:05
 *  Author: user
 */ 

#define ADC_START()	ADCSRA |= (1<<ADSC)
#define ADC_STOP()	ADCSRA &= ~(1<<ADSC)

#include "GP2Y0A02.h"

// 5ms의 CTC로 얽어야 함
// 거리 측정하고 값이 5ms 뒤에 나옴
// PORTF ADC 있음

void ADC_Init(void)
{
	//ADC0 사용, Big Endian
	ADMUX = (1<<REFS0) | (1<<MUX0); // AVCC랑 AREF 되어 있는것 같음
	//128 분주 => 115200
	ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADC_START();
}

uint16_t GP2Y0A_read_ADC()
{
	uint16_t ADC_data =0;
	
	while(!(ADCSRA & (1<<ADIF)));
	ADC_data = ADCL;
	ADC_data |= ADCH<<8;
	return ADC_data;
}
float GP2Y0A_Distance()
{
	uint16_t adcRaw=GP2Y0A_read_ADC();
	float adcVoltage = 0.0f;
	float L = 0.0f;
	adcVoltage = ((float)adcRaw * 5.0f) / 1024.0f;
	
	//Vout = A * (1/L) + B
	//기울기 A = (y2-y1) / (x2-x1)
	//x = 1/L
	// x1 : 1/100, x2 : 1/25
	// y1 : 2.25,   y2 : 0.6
	// A = (2.25 - 0.6) / (0.04 - 0.01) = 55
	// B = y1 - Ax1 = 1.7
	// L = A / (Vout - B)
	float A = 50.0f;
	float B = 0.0;
	L = A / (adcVoltage - B);
	return L;
}
