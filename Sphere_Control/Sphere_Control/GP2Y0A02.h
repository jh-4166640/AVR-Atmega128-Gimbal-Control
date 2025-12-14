/*
 * GP2Y0A02.h
 *
 * Created: 2025-12-14 오전 12:35:36
 *  Author: user
 */ 


#ifndef GP2Y0A02_H_
#define GP2Y0A02_H_


#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

void ADC_Init(void);
uint16_t GP2Y0A_read_ADC();
float GP2Y0A_Distance();




#endif /* GP2Y0A02_H_ */