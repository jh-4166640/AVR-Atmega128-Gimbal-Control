/*
 * uart.c
 *
 * Created: 2025-11-25 오후 12:41:06
 *  Author: user
 */ 

#include "uart.h"

void UART0_Init_Intcon(void) // Interrupt 초기화
{
	// UBRR value (14745600/ (16 * 115200)) - 1
	UBRR0H = (UBRR_VAL >> 8);
	UBRR0L = (UBRR_VAL & 0xFF);
	UCSR0B = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0); // 수신 인터럽트
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);				// N81, 8비트 데이터
}

void UART0_putch(uint8_t data)
{
	while(!(UCSR0A & (1<<UDRE0))); // UDRE0
	UDR0 = data;
}

void UART0_puts(uint8_t *datas)
{
	while (*datas != 0) {
		UART0_putch(*datas++);
	}
}
