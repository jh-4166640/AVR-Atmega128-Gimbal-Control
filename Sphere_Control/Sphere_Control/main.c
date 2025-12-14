/*
 * Sphere_Control.c
 *
 * Created: 2025-11-11 10:02:30AM 
 * Author : Jiheon
 */ 

#include "main.h"


/*
	pitch는 위로 올라가면 -
	yaw는 왼쪽이 +	
*/
#define StabilizeCNT		5
#define StabilizeCNT_p		2
#define AlamLED_PORT		PORTC
#define AlamLED_PITCH_MAX	0
#define AlamLED_YAW_MAX		2
#define AlamLED_Detection	4

void Init_main(void)
{
	char msg[20];
	DDRC = (1<<AlamLED_Detection) | (1<<AlamLED_PITCH_MAX) | (1<<AlamLED_YAW_MAX);
	AlamLED_PORT = 0x00;
	LCD_Init();
	MPU6050_Init();
	ADC_Init();
	uint8_t res = MPU6050_Check_WHO_AM_I();
	if(res == 0)
	{
		LCD_print(0x80, "Pass");
		delay_ms(1000);
	}
	else
	{
		sprintf(msg, "Fail %d",res);
		LCD_print(0x80, msg);
		LCD_print(0x10,"Please Restart");
		while(1);
	}
	Servo_Init();
	servo_status.pitch = 90;
	servo_status.yaw= 90;
	//set_Servo(&servo_status, 0,0);
	set_YawServo(DEG2OCR(servo_status.yaw));
	set_PitchServo(DEG2OCR(servo_status.pitch));
	AlamLED_PORT = (1<<AlamLED_Detection) | (1<<AlamLED_PITCH_MAX) | (1<<AlamLED_YAW_MAX);
}
static inline void AlamLED(uint32_t dist)
{
	if(dist < 50)							AlamLED_PORT &= ~(1<<AlamLED_Detection);
	else										AlamLED_PORT |= (1<<AlamLED_Detection);
	
	if(servo_status.pitch <= SERVO_MIN_PITCH
	|| servo_status.pitch >= SERVO_MAX_PITCH)	AlamLED_PORT &= ~(1<<AlamLED_PITCH_MAX);
	else										AlamLED_PORT |= (1<<AlamLED_PITCH_MAX);
	
	if(servo_status.yaw <= SERVO_MIN_YAW
	|| servo_status.yaw >= SERVO_MAX_YAW)		AlamLED_PORT &= ~(1<<AlamLED_YAW_MAX);
	else										AlamLED_PORT |= (1<<AlamLED_YAW_MAX);
}

int main(void)
{
	cli();
	Init_main();
	sei();

	uint8_t str[32];
	LCD_Clear();
	pitch_deg_stabilize = 0;
	yaw_deg_stabilize = 0;
	uint16_t scan_MPUcnt = 0;
	uint16_t scan_MPUcnt_p = 0;

	while (1)
	{
		if(data_ready_flg)
		{
			data_ready_flg = 0;
			if(scan_MPUcnt == StabilizeCNT)
			{
				//pitch_deg_stabilize = pitch_deg;
				yaw_deg_stabilize = yaw_deg;
				scan_MPUcnt=0;
			}
			if(scan_MPUcnt_p == StabilizeCNT_p)
			{
				pitch_deg_stabilize = pitch_deg;
				scan_MPUcnt_p = 0;
			}
			cli();
			sprintf(str,"P:%d p:%d y:%d",(int16_t)pitch_deg_stabilize,(int16_t)pitch_deg,(int16_t)yaw_deg);
			LCD_print(0x80,str);
			
			set_Servo_angle(&servo_status,pitch_deg,pitch_deg_stabilize,yaw_deg,yaw_deg_stabilize);
			//sprintf(str,"Y:%d p:%d y:%d",(int16_t)yaw_deg_stabilize, servo_status.pitch, servo_status.yaw);
			//LCD_print(0x10,str);	
			 
			float dist = GP2Y0A_Distance();
			uint32_t dist_int = (uint32_t)(dist);
			sprintf(str,"distance %d",dist_int);
			AlamLED(dist_int);
			LCD_print(0x10,str);
			sei();
			scan_MPUcnt++;
			
			_delay_ms(50);
		}
		
		//delay_ms(300);
	}
}
