/*
 * Sphere_Control.c
 *
 * Created: 2025-11-11 10:02:30AM 
 * Author : Jiheon
 */ 

#include "main.h"

int main(void)
{
	LCD_Init();
	GY25_Init();
	Servo_Init();
	sei();
	DDRD = 0x00;
	//Servo_sample_co++de();
	gy25_t angle;
	char str[32];
    while (1) 
    {
		angle = read_GY25();
		LCD_Clear();
		LCD_Pos(0,0);
		sprintf(str,"Y: %3d P: %3d",angle.yaw,angle.pitch);
		LCD_Str(str);
		LCD_Pos(1,0);
		sprintf(str,"R: %3d",angle.roll	);
		LCD_Str(str);
		delay_ms(100);
    }
}

