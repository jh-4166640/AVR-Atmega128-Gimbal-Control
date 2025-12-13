/*
 * MPU6050.c
 *
 * Created: 2025-12-09 오전 9:51:26
 *  Author: user
 */ 

#include "MPU6050.h"
#define MPU6050_I2C_ADDRESS 0x68
#define MPU6050_ADDR_W		0xD0 // 0x68->(1101000*2) + 0 = 0xD0


volatile float twoKp = KpDef;
volatile float twoKi = KiDef;
volatile Quarternion_t Qtn ={
	.q0 = 1.0f,
	.q1 = 0.0f,
	.q2 = 0.0f,
	.q3 = 0.0f,
};
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;


void Timer3_16bit_CTC_Init(void)
{
	// OCR3A = FCPU/(N*delta T)-1
	// N = 64
	//OCR3A = 2303
	// 100 Hz CTC mode
	TCCR3A = 0x00; // CTC, 
	TCCR3B = (1<<WGM32) | (1<<CS31) | (1<<CS30);
	OCR3A = 2303;
	ETIMSK |= (1<<OCIE3A);
}


uint8_t MPU6050_Check_WHO_AM_I(void)
{
    uint8_t device_id = 0;
    uint8_t ret;
	uint8_t str[16];
    ret = TWI_Master_Receive_ExDevice(MPU6050_ADDR_W, MPU6050_WHO_AM_I, &device_id);

    // TWI 통신 자체에 오류가 있었는지 확인 (ret != 0)
    if (ret != 0) {
        return ret; // TWI 에러 코드를 반환
    }
	sprintf(str, "MPU6050 ID %x", device_id);
	LCD_print(0x80,str);
	//LCD_Char(device_id);
    _delay_ms(400);
    // 반환된 ID가 예상 값(0x68)과 일치하는지 확인
    if (device_id == MPU6050_I2C_ADDRESS) {
        return 0; // 성공
    } else {
        return 55; // ID 불일치 오류
    }
}
uint8_t MPU6050_Check_PWR_MGMT(void)
{
	uint8_t device_id = 0;
	uint8_t ret;
	ret = TWI_Master_Receive_ExDevice(MPU6050_ADDR_W, MPU6050_PWR_MGMT_1, &device_id);

	// TWI 통신 자체에 오류가 있었는지 확인 (ret != 0)
	if (ret != 0) {
		return ret; // TWI 에러 코드를 반환
	}
	LCD_Clear();
	LCD_Pos(0,0);
	LCD_Char(device_id);
	_delay_ms(1000);
	return 0;
}
void MPU6050_Init()
{
	Timer3_16bit_CTC_Init();
	Init_TWI();
	_delay_ms(100);
	
	/*MPU6050 Wake-up*/
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W,MPU6050_PWR_MGMT_1,0x80);
	_delay_ms(100);
	/*
	Sampling Rate
	Sampling_Rate = Gyro_output_rate/(1+SMPLRT_DIV)
	Gyro_output_rate = 1kHz(DLPF use)
	-> SMPLRT_DIV = Gyro_output_rate/(Sampling_Rate+1)
	*/
	// 100Hz를 Sampling rate로 할거니까 SMLRT_DIV = 9
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W,MPU6050_SMPLRT_DIV,0x09);
	_delay_ms(100);
	
	/*
	Config
	FSYNC disable : 0bxx000xxx
	DLPF_CFG : 2, Sampling rate => 100Hz이므로
	delay = 3.0ms
	Config = 0bxx000010 : 0x02
	*/
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W,MPU6050_CONFIG,0x02);
	_delay_ms(100);
	
	/* GYRO-CONFIG FS-SEL = 0, 250deg/sec 감도 131.0f*/
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W, MPU6050_GYRO_CONFIG,0x00);
	_delay_ms(100);
	
	/* ACCEL-CONFIG AFS_SEL=1, DHPF=0 감도 8192*/
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W, MPU6050_ACCEL_CONFIG,0x08);
	_delay_ms(100);
	
	/*zero motion dectection 추가할지 말지 고민*/
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W,MPU6050_PWR_MGMT_1,0x0B);
	_delay_ms(100);
	
	TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W,MPU6050_PWR_MGMT_2,0x00);
	_delay_ms(100);
	
	//TWI_Write_SLAW_REG_DATA(MPU6050_ADDR_W, MPU6050_FIFO_EN, 0x78);
	//_delay_ms(100);
}


void MPU6050_read(MPU6050_Data_t *data)
{
	uint8_t buf[15];
	uint8_t ret;
	uint8_t msg[15];
	// Accel X 값부터 지정
	TWI_Start();
	TWI_Write_SLAW(MPU6050_ADDR_W);
	TWI_Write_Data(MPU6050_ACCEL_XOUT_H);
	// 센서 읽기 시작
	TWI_Restart();
	TWI_Write_SLAR(MPU6050_ADDR_W);
	for (uint8_t i= 0; i < 13; i++) {
		ret=TWI_Read_Data(&buf[i]);
		if(ret){
			TWI_Stop();
			sprintf(msg, "read error %d",ret);
			LCD_print(0x80,msg);
			//_delay_ms(1000);
			break;
		}
	}
	TWI_Read_Data_NACK(&buf[13]);
	TWI_Stop();
	data->accelX = (int16_t)((buf[0] << 8) | buf[1]);
	data->accelY = (int16_t)((buf[2] << 8) | buf[3]);
	data->accelZ = (int16_t)((buf[4] << 8) | buf[5]);

	data->gyroX = (int16_t)((buf[8] << 8) | buf[9]);
	data->gyroY = (int16_t)((buf[10] << 8) | buf[11]);
	data->gyroZ = (int16_t)((buf[12] << 8) | buf[13]);
}


void MPU6050_read_FIFO(MPU6050_Data_t *data)
{
	// FIFO에 저장된 한 샘플의 크기 (Accel 6 + Temp 2 + Gyro 6 = 14바이트)
	const uint8_t FIFO_PACKET_SIZE = 14;
	uint8_t buf[FIFO_PACKET_SIZE];
	uint8_t ret;
	uint8_t msg[15];
	
	uint16_t fifo_count; // FIFO에 쌓인 바이트 수
	uint8_t count_buf[2];
	/* FIFO 확인 */
	ret=TWI_Start();							if(ret) return ret;
	ret=TWI_Write_SLAW(MPU6050_ADDR_W);			if(ret) return ret;
	ret=TWI_Write_Data(MPU6050_FIFO_COUNTH);	if(ret) return ret;
	ret=TWI_Restart();							if(ret) return ret;
	ret=TWI_Write_SLAR(MPU6050_ADDR_W);			if(ret) return ret;
	ret=TWI_Read_Data(&count_buf[0]);			if(ret) return ret;
	ret=TWI_Read_Data_NACK(&count_buf[1]);		if(ret) return ret;
	TWI_Stop();

	fifo_count = (count_buf[0] << 8) | count_buf[1];
	if (fifo_count < FIFO_PACKET_SIZE) {
		// FIFO에 데이터가 충분히 쌓이지 않았음 (읽을 데이터가 없음)
		sprintf(msg, "FIFO Count: %d", fifo_count);
		LCD_print(0x80, msg);
		_delay_ms(100);
		return; // 데이터 읽기 실패
	}
	
	// 1. FIFO R/W 레지스터 (0x74)를 시작 주소로 지정
	TWI_Start();
	TWI_Write_SLAW(MPU6050_ADDR_W);
	TWI_Write_Data(MPU6050_FIFO_R_W); // FIFO 버퍼의 읽기/쓰기 주소

	// 2. 센서 읽기 시작 (반복 시작)
	TWI_Restart();
	TWI_Write_SLAR(MPU6050_ADDR_W);
	
	// 3. FIFO 버퍼에서 14바이트 연속 읽기 (Burst Read)
	for (uint8_t i = 0; i < FIFO_PACKET_SIZE - 1; i++) {
		ret = TWI_Read_Data(&buf[i]); 
		if(ret){
			TWI_Stop();
			sprintf(msg, "read error %d",ret);
			LCD_print(0x80,msg);
			_delay_ms(1000);
			break;
		}
	}
	TWI_Read_Data_NACK(&buf[FIFO_PACKET_SIZE - 1]);
	TWI_Stop();
	
	
	data->accelX = (int16_t)((buf[0] << 8) | buf[1]);
	data->accelY = (int16_t)((buf[2] << 8) | buf[3]);
	data->accelZ = (int16_t)((buf[4] << 8) | buf[5]);
	// buf[6], buf[7]은 온도 데이터 (TEMP)
	data->gyroX = (int16_t)((buf[8] << 8) | buf[9]);
	data->gyroY = (int16_t)((buf[10] << 8) | buf[11]);
	data->gyroZ = (int16_t)((buf[12] << 8) | buf[13]);
}


void MahonyAHRSupdateIMU(Quarternion_t *filter,float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = Qtn.q1 * Qtn.q3 - Qtn.q0 * Qtn.q2;
		halfvy = Qtn.q0 * Qtn.q1 + Qtn.q2 * Qtn.q3;
		halfvz = Qtn.q0 * Qtn.q0 - 0.5f + Qtn.q3 * Qtn.q3;
		
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / SamplingFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / SamplingFreq);
			integralFBz += twoKi * halfez * (1.0f / SamplingFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / SamplingFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / SamplingFreq));
	gz *= (0.5f * (1.0f / SamplingFreq));
	qa = Qtn.q0;
	qb = Qtn.q1;
	qc = Qtn.q2;
	Qtn.q0 += (-qb * gx - qc * gy - Qtn.q3 * gz);
	Qtn.q1 += (qa * gx + qc * gz - Qtn.q3 * gy);
	Qtn.q2 += (qa * gy - qb * gz + Qtn.q3 * gx);
	Qtn.q3 += (qa * gz + qb * gy - qc * gx);
	
	// Normalise quaternion
	recipNorm = InvSqrt(Qtn.q0 * Qtn.q0 + Qtn.q1 * Qtn.q1 + Qtn.q2 * Qtn.q2 + Qtn.q3 * Qtn.q3);
	Qtn.q0 *= recipNorm;
	Qtn.q1 *= recipNorm;
	Qtn.q2 *= recipNorm;
	Qtn.q3 *= recipNorm;
	
	
	filter->q0 = Qtn.q0;
	filter->q1 = Qtn.q1;
	filter->q2 = Qtn.q2;
	filter->q3 = Qtn.q3;
}

float InvSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
