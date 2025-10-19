/*
 * Exam12_1.c (ATmega128 TWI Master)
 * TWI함수에서 error 검사하여 수행할 경우 _USE_SAFTY_TWI_를 정의하고 사용
 * _USE_SAFTY_TWI_가 정의되지 않은 경우: error 검출 없음
 * 일반적인 경우에는 즉, 마스터/슬레이브 등 AVR 내부 기능만 사용할 경우에는 정의할 필요 없음.
 * _USE_SAFTY_TWI_가 정의된 경우: error 검출하면서 TWI 구동 
 * Created: 2025-08-27 오후 1:53:49
 * Author : ehlee
 */ 

/* PC 포트의 스위치 상태에 따라 다음과 같이 동작
 * - PC0: hold -> send 'A' repeatedly (polling)
 * - PC1: hold -> send A..Z repeatedly (polling)
 * - PC2: edge -> send predefined string once (polling)
 * - PC3: falling edge (INT3) -> (ISR triggers flag) send predefined string
 * LCD first line: "Master Send :"
 * LCD second line: last sent characters
 *
 * Address format: 8-bit (7-bit<<1 | R/W=0) -> use 0x02 to match slave TWAR.
 */

/*
 * =================================================_
 * M A I N __ M A S T E R P _ R O J E C T  F I L E
 * =================================================_
 */
#define _USE_SAFTY_TWI_
#define F_CPU 14745600UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "twi_gcc.h"
#include "lcd_gcc.h"
#include "usart_gcc.h"

// Slave address for TWI communication
#define SLAVE_ADDR 0x08              // 8-bit base (W=0)

// PC0~PC3 정보 및 마스크 패턴
enum _PC_PUSH_BUTTON {
	PC_BTN_0 = 0, PC_BTN_1 = 1, PC_BTN_2 = 2, PC_BTN_3 = 3,
	PC_BTN_4 = 4, PC_BTN_5 = 5, PC_BTN_6 = 6, PC_BTN_7 = 7
};
// used pc port button mask, pc3 just reserved for any other case
#define BUTTON_MASK (_BV(PC_BTN_0)|_BV(PC_BTN_1)|_BV(PC_BTN_2)|_BV(PC_BTN_3))

// Key related 상태 정의
enum _KEY_RELATED {
	KEY_MASK = 0X0F, KEY_RELEASED_STATE = 0x0F, DEBOUNCE_MS = 20,
	NOT_PRESSED_ANY_KEY = -1
};

// FSM states
enum _PROGRAM_STATUS {
	STATUS_IDLE = -1,
	STATUS_SEND_A = 0,
	STATUS_SEND_A_TO_Z = 1,
	STATUS_SEND_STRING_POLLING = 2,
	STATUS_SEND_STRING_INT = 3,
	// STATUS_SEND_A_ABORT = 99
};

#define USART_BUFFER_SIZE 255
// LCD related
enum _LCD_RELATED {
	LCD_FIRST_ROW_LINE = 0, LCD_SECOND_ROW_LINE = 1, // row
	LCD_MAX_DISPLAYABLE_CHARS = 32, LCD_ROW_LINE_MAX_LENGTH = 16 // columns
};

// peroid for timer1 interrupt
#define TIM_1_PERIOD_MS 100 // Timer1 interrupt(100ms 마다 key scanning)
// peroid for timer3 interrupt
#define TIM_3_PERIOD_MS 10 // Timer3 interrupt(i2c를 이용하여 slave에게 글자를 전송하는 주기 조정:10ms tick )

// PC Function sending period
#define SENDING_PERIOD_SEC 1 // period for sending
#define SENDING_PERIOD_100MS 1

// slave LCD clear command
#define SLAVE_LCD_CLEAR_CMD 0X33

// welcome message
static const char welcome_msg_1[] = "AVR TWI Master";
static const char welcome_msg_2[] = "press PC0..PC2";

volatile uint8_t global_lcd_row_0[LCD_ROW_LINE_MAX_LENGTH] = {0};
volatile uint8_t global_lcd_row_1[LCD_ROW_LINE_MAX_LENGTH] = {0};
volatile uint8_t global_twi_send_msg_buffer[LCD_MAX_DISPLAYABLE_CHARS] = {0};
volatile uint8_t global_usart_send_msg_buffer[USART_BUFFER_SIZE] = {0};
static volatile int8_t global_newly_pressed_key = NOT_PRESSED_ANY_KEY;
volatile uint16_t global_timer_sec = 0;
volatile uint16_t global_timer_100ms = 0;

void PORTC_Init(void)
{
	// PC0..PC3 as input with pull-ups
	DDRC &= ~BUTTON_MASK; // Set PC0-PC3 as input
	PORTC |= BUTTON_MASK; // Enable pull-ups on PC0-PC3
}

void DISPLAY_WELCOME_MESSAGE(void)
{
	LCD_Clear();
	LCD_Pos(0,0);
	LCD_Str(welcome_msg_1);
	LCD_Pos(1,0);
	LCD_Str(welcome_msg_2);
}

void Timer1_Init(void)
{
	// Set CTC mode (Clear Timer on Compare Match) -> WGM12=1
	TCCR1B |= (1 << WGM12);
	// Set prescaler to 256
	TCCR1B |= (1 << CS12);
	TCCR1B &= ~((1 << CS11) | (1 << CS10)); // Clear other prescaler bits

	// Set compare value based on TIM_1_PERIOD_MS using integer arithmetic
	// Formula: (F_CPU * Period_in_ms) / (Prescaler * 1000) - 1
	OCR1A = (uint16_t)((F_CPU * TIM_1_PERIOD_MS / 256 / 1000) - 1);

	// Enable Timer1 Compare A Match interrupt
	TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
//Timer 1 INT.: PC 포트를 SCAN하여 key 눌림 상태를 체크하고, 
// 키 정보를 알려줌. (checked only every TIM_1_PERIOD_MS)
{
	static uint8_t prev_key_state = KEY_RELEASED_STATE;
	uint8_t current_key_state = PINC & KEY_MASK;

	if (prev_key_state == KEY_RELEASED_STATE && current_key_state != KEY_RELEASED_STATE)
	{
		// A key has been pressed, find out which one
		for (int8_t i = PC_BTN_0; i <= PC_BTN_3; i++)
		{
			if (!(current_key_state & (1 << i)))
			{
				global_newly_pressed_key = i; // Set global variable
				break;
			}
		}
	}
	prev_key_state = current_key_state;
}

void Timer3_Init(void)
// 10msec INT
{
	// Set CTC mode (Clear Timer on Compare Match) -> WGM32=1
	TCCR3B |= (1 << WGM32);
	// Set prescaler to 1024
	TCCR3B |= (1 << CS32) | (1 << CS30);

	// Set compare value for 10ms
	OCR3A = (uint16_t)((F_CPU * TIM_3_PERIOD_MS / 1024 / 1000) - 1);

	// Enable Timer3 Compare A Match interrupt
	ETIMSK |= (1 << OCIE3A);  // ATmega128 ETIMSK
}

ISR(TIMER3_COMPA_vect)
// 송신용 비교일치 인터럽트
{
	static uint16_t timer3_10ms_cnt = 0;
	timer3_10ms_cnt++;

	if(timer3_10ms_cnt >= 100)
	{
		global_timer_sec++;
		timer3_10ms_cnt = 0;
	}

	if(timer3_10ms_cnt % 10 == 0)
	{
		global_timer_100ms++;
	}
}

// clear the specific row of LCD
void LCD_Clear_one_row(uint8_t param_selected_row)
{
	uint8_t local_value_1 = 0;
	LCD_Pos(param_selected_row, 0);

	for(local_value_1 = 0; local_value_1 < 16; local_value_1++)
	{
		LCD_Char(' ');
	}
	LCD_Pos(param_selected_row, 0); // move cursor to the beginning of the row
}



static inline uint8_t key_is_pressed(uint8_t bit){
    return ( (PINC & _BV(bit)) == 0 );	// active-low
}

// --- Helper function to send a string via TWI ---
uint8_t TWI_Master_Transmit_String(const char* str, uint8_t SlaveAddr)
{
	uint8_t ret;
	if ((ret = TWI_Start()) != 0) {
		TWI_Stop();
		return ret;
	}
	if ((ret = TWI_Write_SLAW(SlaveAddr)) != 0) {
		TWI_Stop();
		return ret;
	}
	while (*str) {
		if ((ret = TWI_Write_Data(*str++)) != 0) {
			TWI_Stop();
			return ret;
		}
	}
	TWI_Stop();
	return 0; // Success
}

void process_pc_button_0(int8_t param_program_status, int8_t param_previous_program_status)
// 500msec 마다 "A" 전송
{
	// Init. local variables
	uint8_t twi_result = 0;
	static uint8_t send_count = 0;
	static uint16_t last_execution_time = 0;
	static uint8_t local_lcd_column_pose = 0;

	// 함수 중복 호출 혹은 다른 실행 함수에서 넘어온 경우
	// static 변수로 선언되어 저장된 변수들을 초기화하는 과정
	if((param_program_status != param_previous_program_status))
	{
		// reset static variables when not in STATUS_SEND_A
		send_count = 0;
		// last_execution_time = 0;
		local_lcd_column_pose = 0;

		LCD_Clear_one_row(0);
		LCD_Clear_one_row(1);
		puts_USART1("PC0 reset\r\n");
		param_previous_program_status = param_program_status;
	}
	else // continue
	
	// if(global_timer_sec - last_execution_time >= SENDING_PERIOD_SEC) // every SENDING_PERIOD_SEC seconds(1sec)
	if(global_timer_100ms - last_execution_time >= SENDING_PERIOD_100MS * 5) // every SENDING_PERIOD_100MS seconds
	{
		// last_execution_time = global_timer_sec;
		last_execution_time = global_timer_100ms;
		twi_result = TWI_Master_Transmit('A', SLAVE_ADDR);
		// sprintf((char*)global_usart_send_msg_buffer, "%3d sec\r\n", global_timer_sec);
		// puts_USART1((char*)global_usart_send_msg_buffer); // send current seconds to USART1
		if(twi_result == 0)
		// 16번째 글자 전송 시 lcd 1 row 초기화 및 (1,0) 커서 좌표 이동
		// 그리고 17번째 글자 출력 후 커서 좌표++을 수행함.
		{
			LCD_Pos(0, 0);
			sprintf((char*)global_lcd_row_1, "TX count: %d", ++send_count);
			LCD_Str((char*)global_lcd_row_1);
			
			if(local_lcd_column_pose >= 16)
			{
				LCD_Clear_one_row(1);
				local_lcd_column_pose = 0;
				LCD_Pos(1, local_lcd_column_pose);
				LCD_Char('A');
				local_lcd_column_pose += 1;
			}
			else
			{
				LCD_Pos(1, local_lcd_column_pose); // display current alphabet at row 1
				LCD_Char('A');
				local_lcd_column_pose += 1;
			}
		}
		else // twi error
		{
			// Error occurred during transmission
			LCD_Clear_one_row(1);
			LCD_Clear_one_row(0);
			LCD_Str("TWI TX Error");
		}
	}else;
}

void process_pc_button_1(int8_t param_program_status, int8_t param_previous_program_status)
// 매 500msec 마다 "A" ~ "Z" 알파벳 전송
{
	uint8_t twi_result = 0;
	static uint8_t send_count = 0;
	static uint16_t last_execution_time = 0;
	static uint8_t local_lcd_column_pose = 0;
	static uint8_t character_shifter = 0;
	
	if((param_program_status != param_previous_program_status) )
	{
		// reset static variables when not in STATUS_SEND_A
		send_count = 0;
		// last_execution_time = 0;
		local_lcd_column_pose = 0;
		character_shifter = 0;

		LCD_Clear_one_row(0);
		LCD_Clear_one_row(1);
		puts_USART1("PC1 reset\r\n");
		param_previous_program_status = param_program_status;
	}
	else // continue
	
	// if(global_timer_sec - last_execution_time >= SENDING_PERIOD_SEC) // every SENDING_PERIOD_SEC seconds
	if(global_timer_100ms - last_execution_time >= SENDING_PERIOD_100MS * 5) // every SENDING_PERIOD_100MS seconds
	{
		// last_execution_time = global_timer_sec;
		last_execution_time = global_timer_100ms;
		if(character_shifter >= 26) character_shifter = 0; // wrap around after 'Z'
		twi_result = TWI_Master_Transmit('A'+(character_shifter), SLAVE_ADDR);
		// sprintf((char*)global_usart_send_msg_buffer, "%3d sec\r\n", global_timer_sec);
		// puts_USART1((char*)global_usart_send_msg_buffer); // send current seconds to USART1
		if(twi_result == 0)
		{
			LCD_Pos(0, 0);
			sprintf((char*)global_lcd_row_1, "TX count: %d", ++send_count);
			LCD_Str((char*)global_lcd_row_1);
			
			if(local_lcd_column_pose >= 16)
			{
				LCD_Clear_one_row(1);
				local_lcd_column_pose = 0;
				LCD_Pos(1, local_lcd_column_pose);
				LCD_Char('A'+(character_shifter));
				local_lcd_column_pose += 1;
				character_shifter += 1;
			}
			else
			{
				LCD_Pos(1, local_lcd_column_pose); // display current alphabet at row 1
				LCD_Char('A'+(character_shifter));
				local_lcd_column_pose += 1;
				character_shifter += 1;
			}
		}
		else // twi error
		{
			// Error occurred during transmission
			LCD_Clear_one_row(1);
			LCD_Clear_one_row(0);
			LCD_Str("TWI TX Error");
		}
	}else;
}

void process_pc_button_2(int8_t param_program_status, int8_t param_previous_program_status)
// 매 500msec마다 문자열 "Tech University of KOREA, Department of electronic"을 순차적으로 전송
{
	uint8_t twi_result = 0;
	static uint8_t send_count = 0;
	static uint16_t last_execution_time = 0;
	static uint8_t local_lcd_column_pose = 0;
	static uint8_t character_shifter = 0;
	static char predefined_string[] = "Tech University of KOREA, Department of electronic ";

	if((param_program_status != param_previous_program_status) )
	{
		// reset static variables when not in STATUS_SEND_A
		send_count = 0;
		// last_execution_time = 0;
		local_lcd_column_pose = 0;
		character_shifter = 0;

		LCD_Clear_one_row(0);
		LCD_Clear_one_row(1);
		puts_USART1("PC2 reset\r\n");
		param_previous_program_status = param_program_status;
	}
	else // continue
	
	// if(global_timer_sec - last_execution_time >= SENDING_PERIOD_SEC) // every SENDING_PERIOD_SEC seconds
	if(global_timer_100ms - last_execution_time >= SENDING_PERIOD_100MS * 5) // every SENDING_PERIOD_100MS seconds
	{
		// last_execution_time = global_timer_sec;
		last_execution_time = global_timer_100ms;
		if(character_shifter >= strlen(predefined_string)) character_shifter = 0;
		// wrap around after end of string, strlen을 이용하여 글자수를 계산하여 현재 shifter가 문자의 수보다 넘지 못하도록 방지함.
		twi_result = TWI_Master_Transmit(predefined_string[character_shifter], SLAVE_ADDR);
		// sprintf((char*)global_usart_send_msg_buffer, "%3d sec\r\n", global_timer_sec);
		// puts_USART1((char*)global_usart_send_msg_buffer); // send current seconds to USART1
		if(twi_result == 0)
		{
			LCD_Pos(0, 0);
			sprintf((char*)global_lcd_row_1, "TX count: %d", ++send_count);
			LCD_Str((char*)global_lcd_row_1);
			
			if(local_lcd_column_pose >= 16)
			{
				LCD_Clear_one_row(1);
				local_lcd_column_pose = 0;
				LCD_Pos(1, local_lcd_column_pose);
				LCD_Char(predefined_string[character_shifter]);
				local_lcd_column_pose += 1;
				character_shifter += 1;
			}
			else
			{
				LCD_Pos(1, local_lcd_column_pose); // display current alphabet at row 1
				LCD_Char(predefined_string[character_shifter]);
				local_lcd_column_pose += 1;
				character_shifter += 1;
			}
		}
		else // twi error
		{
			// Error occurred during transmission
			LCD_Clear_one_row(1);
			LCD_Clear_one_row(0);
			LCD_Str("TWI TX Error");
		}
	}else;
}

int main(void){
	// LCD init
	LCD_Init();
	// welcome message
	DISPLAY_WELCOME_MESSAGE();
	// TWI initialize
	Init_TWI();        // uses fixed TWBR/TWSR preset from header
	// PC0..PC3 as input with pull-ups
	PORTC_Init();
	// USART init
	Init_USART1(0);
	// Initialize Timer
	Timer1_Init();
	// Timer2_Init();
	Timer3_Init();

	int8_t program_status = STATUS_IDLE;
	int8_t previous_program_status = STATUS_IDLE;
	// int8_t previous_program_status = STATUS_IDLE;

	// global interrupt enable
	sei();

	while(1)
	{
		// Check if the ISR has detected a new key press
		if (global_newly_pressed_key != NOT_PRESSED_ANY_KEY)
		{
			// sprintf((char*)global_usart_send_msg_buffer, "%d %d\r\n", program_status, previous_program_status);
			// puts_USART1((char*)global_usart_send_msg_buffer); // send current seconds to USART1

			int8_t key_to_process = global_newly_pressed_key;
			// Reset the global key variable so we don't process the same key again
			global_newly_pressed_key = NOT_PRESSED_ANY_KEY;

			switch (key_to_process)
			{
				case PC_BTN_0:
				// TODO: Add function for key 1
				// previous_program_status = program_status;
				if(program_status == STATUS_SEND_A) previous_program_status = STATUS_IDLE; // sensing the re-entry
				program_status = STATUS_SEND_A;
				LCD_Clear();
				LCD_Pos(0, 0);
				LCD_Str("pc 0 Pressed");
				TWI_Master_Transmit(SLAVE_LCD_CLEAR_CMD, SLAVE_ADDR); // command to clear slave lcd
				break;
				case PC_BTN_1:
				// TODO: Add function for key 2
				// previous_program_status = program_status;
				if(program_status == STATUS_SEND_A_TO_Z) previous_program_status = STATUS_IDLE; // sensing the re-entry
				program_status = STATUS_SEND_A_TO_Z;
				LCD_Clear();
				LCD_Pos(0, 0);
				LCD_Str("pc 1 Pressed");
				TWI_Master_Transmit(SLAVE_LCD_CLEAR_CMD, SLAVE_ADDR); // command to clear slave lcd
				break;
				case PC_BTN_2:
				// TODO: Add function for key 3
				// previous_program_status = program_status;
				if(program_status == STATUS_SEND_STRING_POLLING) previous_program_status = STATUS_IDLE; // sensing the re-entry
				program_status = STATUS_SEND_STRING_POLLING;
				LCD_Clear();
				LCD_Pos(0, 0);
				LCD_Str("pc 2 Pressed");
				TWI_Master_Transmit(SLAVE_LCD_CLEAR_CMD, SLAVE_ADDR); // command to clear slave lcd
				break;
				case PC_BTN_3: // this is just reserved for any other case
				// TODO: Add function for key 4
				// previous_program_status = program_status;
				program_status = STATUS_IDLE;
				LCD_Clear();
				LCD_Pos(0, 0);
				LCD_Str("pc 3 Pressed");
				TWI_Master_Transmit(SLAVE_LCD_CLEAR_CMD, SLAVE_ADDR); // command to clear slave lcd
				break;
				default: // Should not reach here
				break;
			} // end of switch case for key functions
			sprintf((char*)global_usart_send_msg_buffer, "%d %d\r\n", program_status, previous_program_status);
			puts_USART1((char*)global_usart_send_msg_buffer); // send current seconds to USART1
		} // end of if for key pressed

		if(program_status == STATUS_SEND_A)
		{
			process_pc_button_0(program_status, previous_program_status);
			previous_program_status = program_status;
		}
		else if(program_status == STATUS_SEND_A_TO_Z)
		{
			process_pc_button_1(program_status, previous_program_status);
			previous_program_status = program_status;
		}
		else if(program_status == STATUS_SEND_STRING_POLLING)
		{
			process_pc_button_2(program_status, previous_program_status);
			previous_program_status = program_status;
		}
		else; // do nothing in idle or other states

	} // end of while
	return 0; // end of program
} // end of main