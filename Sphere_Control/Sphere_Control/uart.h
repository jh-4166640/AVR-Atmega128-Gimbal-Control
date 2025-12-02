/*
 * uart.h
 *
 * Created: 2025-11-25 오후 12:09:37
 *  Author: jiheon choi
 */ 

#ifndef UART_H_
#define UART_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define BAUD			115200
#define UBRR_VAL		((14745600UL/16/BAUD)-1)



void UART0_Init_Intcon(void); // Interrupt 초기화
void UART0_putch(uint8_t data);
void UART0_puts(uint8_t *data);


#endif /* UART_H_ */