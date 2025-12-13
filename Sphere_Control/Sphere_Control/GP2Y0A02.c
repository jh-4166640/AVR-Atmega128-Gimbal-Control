/*
 * GP2Y0A02.c
 *
 * Created: 2025-12-14 오전 12:36:05
 *  Author: user
 */ 

#define ADC_START	ADCSRA |= (1<<ADSC)
#define ADC_STOP	ADCSRA &= ~(1<<ADSC)

#include "GP2Y0A02.h"

// 5ms의 CTC로 얽어야 함
// 거리 측정하고 값이 5ms 뒤에 나옴
// PORTF ADC 있음

void ADC_Init(void)
{
	//ADC0 사용, Big Endian
	ADMUX = (1<<REFS0); // AVCC랑 AREF 되어 있는것 같음
	ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE);
}