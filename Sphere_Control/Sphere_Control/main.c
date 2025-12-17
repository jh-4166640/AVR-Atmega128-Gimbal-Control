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
#define RotateLedA			0
#define RotateLedB			2
#define RotateLedC			4
#define AlamLED_Detection	6

void Init_main(void)
{
	char msg[20];
	DDRC = (1<<AlamLED_Detection) | (1<<RotateLedA) | (1<<RotateLedB) | (1<<RotateLedC);
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
	AlamLED_PORT = (1<<AlamLED_Detection) | (1<<RotateLedA) | (1<<RotateLedB) | (1<<RotateLedC);
}
static inline uint8_t Output_ABC(uint8_t num)
{
	uint8_t out = 0x00;
	if(num == 1) out|=(1<<RotateLedA);
	else if(num == 2) out = (1<<RotateLedB);
	else if(num == 3) out = (1<<RotateLedB) | (1<<RotateLedA);
	else if(num == 4) out = (1<<RotateLedC);
	else if(num == 5) out = (1<<RotateLedC) | (1<<RotateLedA);
	else if(num == 6) out = (1<<RotateLedC) | (1<<RotateLedB);
	return out;
}
static inline void RotateLED_Calc(uint16_t servo_yaw_deg)
{
	//#define SERVO_MAX_YAW		150
	//#define SERVO_MIN_YAW		30
	if(servo_yaw_deg >= SERVO_MAX_YAW-20) //150~130
	{
		AlamLED_PORT&=(1<<AlamLED_Detection);
		AlamLED_PORT|=Output_ABC(1);
	}
	else if(servo_yaw_deg < SERVO_MAX_YAW-20 &&  servo_yaw_deg >= SERVO_MAX_YAW-40) //129~110
	{
		AlamLED_PORT&=(1<<AlamLED_Detection);
		AlamLED_PORT|=Output_ABC(2);	
	}
	else if(servo_yaw_deg < SERVO_MAX_YAW-40 &&  servo_yaw_deg >= SERVO_MAX_YAW-80) //109~70
	{
		AlamLED_PORT&=(1<<AlamLED_Detection);
		AlamLED_PORT|=Output_ABC(3);
	}
	else if(servo_yaw_deg < SERVO_MAX_YAW-80 &&  servo_yaw_deg >= SERVO_MAX_YAW-100) //69~50
	{
		AlamLED_PORT&=(1<<AlamLED_Detection);
		AlamLED_PORT|=Output_ABC(4);
	}
	else if(servo_yaw_deg < SERVO_MAX_YAW-100 &&  servo_yaw_deg >= SERVO_MIN_YAW) //49~30
	{
		AlamLED_PORT&=(1<<AlamLED_Detection);
		AlamLED_PORT|=Output_ABC(5);
	}
	
}
static inline void AlamLED(uint32_t dist, uint16_t servo_yaw_deg)
{
	if(dist < 50)							AlamLED_PORT &= ~(1<<AlamLED_Detection);
	else									AlamLED_PORT |= (1<<AlamLED_Detection);
	RotateLED_Calc(servo_yaw_deg);
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
			AlamLED(dist_int,servo_status.yaw);
			LCD_print(0x10,str);
			sei();
			scan_MPUcnt++;
			
			_delay_ms(50);
		}
		
		//delay_ms(300);
	}
}
