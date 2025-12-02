/*
 * gy25.c
 *
 * Created: 2025-11-25 오후 12:54:11
 *  Author: user
 */ 

#include "gy25.h"

ISR(USART0_RX_vect)
{
	uint8_t temp = UDR0;
	if(temp == FRAME_START)
	{
		gy25_idx = 0;
		gy25_buffer[gy25_idx++] = temp;
	}
	else if(gy25_idx < 8)
	{
		gy25_buffer[gy25_idx++] = temp;
		if(gy25_idx == 8) gy25_idx =0;
	}
	else gy25_idx = 0;
}

void GY25_Init(void)
{
	gy25_idx=0;
	UART0_Init_Intcon();
	
	uint16_t init_ms = 4000;
	LCD_Clear();
	LCD_Pos(0,0);
	LCD_Str(MSG_FIRST_CORR0);
	LCD_Pos(1,0);
	LCD_Str(MSG_FIRST_CORR1);
	_delay_ms(4000);
	
	/* Correction pitch/roll */
	LCD_Clear();
	LCD_Pos(0,0);
	LCD_Str(MSG_PITCH_CORR);
	LCD_Pos(1,0);
	LCD_Str(MSG_WAIT);
	_delay_ms(800);
	LCD_Pos(1,0);
	LCD_Str(MSG_CALIBRATING);
	UART0_putch(COMMAND_START); // GY-25로 메세지 전송 시작
	UART0_putch(CORRECTION_MODE_PITCH); // pitch 보정
	_delay_ms(4000);
	
	/* Correction YAW */
	LCD_Clear();
	LCD_Pos(0,0);
	LCD_Str(MSG_YAW_CORR);
	LCD_Pos(1,0);
	LCD_Str(MSG_WAIT);
	_delay_ms(800);
	LCD_Pos(1,0);
	LCD_Str(MSG_CALIBRATING);
	UART0_putch(COMMAND_START);	// GY-25로 메세지 전송 시작
	UART0_putch(CORRECTION_MODE_YAW);	// yaw(hedding) 보정
	_delay_ms(4000);
}

gy25_t read_GY25(void)
{
	gy25_t data;
	uint8_t buf[8];

	UART0_putch(COMMAND_START);
	UART0_putch(QUERY_MODE);
	_delay_ms(50);
	for(int8_t i = 0;i<8;i++)
	{
		buf[i] = gy25_buffer[i];
	}
	if(buf[FRAME_S] == FRAME_START && buf[FRAME_E] == FRAME_END) // 데이터 정상 수신 완료
	{
		data.yaw = ((buf[YAW_H] << 8) | buf[YAW_L]) / 100;
		data.pitch = ((buf[PITCH_H] << 8) | buf[PITCH_L]) / 100;
		data.roll = ((buf[ROLL_H] << 8) | buf[ROLL_L]) / 100;	
	}
	else
	{
		data.pitch = ERROR_VAL;
		data.roll = ERROR_VAL;
		data.yaw = ERROR_VAL;
	}
	
	return data;
}