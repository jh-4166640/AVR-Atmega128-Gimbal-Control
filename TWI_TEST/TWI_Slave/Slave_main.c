/*
 * Exam12_1: TWI Slave.c
 * - Displays each received byte at "Rec :" line (1st line)
 * - Second line prints incoming stream (clears when full)
 * - Uses SAFE-mode APIs from "twi_gcc.h" (compiled with _USE_SAFTY_TWI_)
 *   receiving ONE byte per TWI transaction (Master_Transmit per char).
 * Author : ehlee
 */

#define _USE_SAFTY_TWI_
#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "twi_gcc.h"
#include "lcd_gcc.h"

#define SLAVE_ADDR 0x08 // 8-bit base (TWAR expects 7-bit<<1)
#define SLAVE_LCD_CLEAR_CMD 0X33 // slave lcd clear command

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

int main(void){
    uint8_t twi_received_data = 0;
    uint8_t lcd_column_pos = 0;

    LCD_Init();
    LCD_Clear();
    LCD_Pos(0,0);
    LCD_Str("TWI Rec :");

    Init_TWI();
    Init_TWI_Slaveaddr(SLAVE_ADDR);

    while(1)
    {
	    if(TWI_Slave_Receive(&twi_received_data) == 0)
	    {
		    if(twi_received_data == SLAVE_LCD_CLEAR_CMD)
		    {
			    LCD_Clear_one_row(1); lcd_column_pos = 0;
		    }
		    else
		    {
			    LCD_Pos(1, lcd_column_pos);
			    LCD_Char(twi_received_data);
			    if(++lcd_column_pos > 16)
			    {
				    // LCD_Pos(1, 0);
				    // for(uint8_t i=0; i<16; i++) LCD_Char(' ');
				    // lcd_column_pos = 0;
				    LCD_Clear_one_row(1);
				    lcd_column_pos = 0;
				    LCD_Char(twi_received_data);
				    lcd_column_pos += 1;
			    }
		    }
	    }
    }
    return 0;
}
