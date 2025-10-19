#ifndef _INCLUDE_TWI_H__
#define _INCLUDE_TWI_H__
// AVR-GCC / Microchip Studio compatible TWI (I2C) helpers for ATmega128
// - Fixed include guards (original had a typo)
// - Header-only via static inline to avoid multiple definition
// - Keeps original APIs and semantics (incl. _USE_SAFTY_TWI_ switch)
// - Uses <avr/io.h> and <stdint.h>
//
// NOTE: Original assumes 8-bit I2C address value (7-bit<<1 | R/W).
//       We preserve that behavior for drop-in compatibility.

#include <avr/io.h>
#include <stdint.h>

// Optional: set default timeout count for safe mode (can override in project)
#ifndef ExtDev_ERR_MAX_CNT
#define ExtDev_ERR_MAX_CNT 2000u
#endif

// ---- Status Codes (Same as original) ----
// Master
#define TWI_START       0x08		// Start 전송에 대한 상태코드
#define TWI_RESTART     0x10		// Restart 전송에 대한 상태코드
#define MT_SLA_ACK      0x18		// SLA+W 패킷 전송에 대한 상태코드
#define MT_DATA_ACK     0x28		// 데이터 패킷 전송에 대한 상태코드
#define MR_SLA_ACK      0x40		// SLA+R 패킷 전송에 대한 상태코드
#define MR_DATA_ACK     0x50		// 데이터 패킷 수신에 대한 ACK 상태코드
#define MR_DATA_NACK    0x58		// 데이터 패킷 수신에 대한 NACK 상태코드
// Slave Receiver
#define SR_SLA_ACK      0x60
#define SR_STOP         0xA0
#define SR_DATA_ACK     0x80
#define SR_DATA_NACK    0x58

// ---- Init helpers (preserve original register values) ----
static inline void Init_TWI(void)
{
    // SCL ~100kHz @ F_CPU=14.7456MHz, prescaler=1
    TWSR = 0x00;
    TWBR = 0x32;
    TWCR = (1<<TWEN);		//TWI Enable
}

static inline void Init_TWI_400K(void)
{
    // SCL ~400kHz @ F_CPU=14.7456MHz, prescaler=1
	// SCL = (Fsck)/(16 + (2*10)) = 409,600
    TWSR = 0x00;
    TWBR = 0x0A;			//SCL 약 400kHz
    TWCR = (1<<TWEN);
}

// ================= SAFE MODE =================
#if defined(_USE_SAFTY_TWI_)
// 신호 전송 완료 검사 및 Status 확인 + Timeout Check 
// Error code: 0=no error, 1=timeout, 2=status mismatch

static inline uint8_t TWI_TransCheck_ACK(uint8_t Stat)
{
    uint16_t cnt = 0;
    while (!(TWCR & (1<<TWINT)))				// 패킷 전송 완료될 때 까지 wait
	{				
        if (cnt++ > ExtDev_ERR_MAX_CNT) return 1; // timeout
    }
    if ((TWSR & 0xF8) != Stat) return 2;         // status mismatch, 전송 검사 error(2 반환)
    return 0;
}

// START
static inline uint8_t TWI_Start(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);	// START 신호 보내기
    return TWI_TransCheck_ACK(TWI_START);
}

// SLA+W
static inline uint8_t TWI_Write_SLAW(uint8_t Addr)
{
    TWDR = Addr;							// 8-bit slave address (W bit assumed in Addr)
    TWCR = (1<<TWINT) | (1<<TWEN);			// SLA + W 패킷 보내기  
    return TWI_TransCheck_ACK(MT_SLA_ACK);  // ACK 수신 후 Status 확인
}

// DATA (write), 데이터 패킷 전송
static inline uint8_t TWI_Write_Data(uint8_t Data)
{
    TWDR = Data;
    TWCR = (1<<TWINT) | (1<<TWEN);		   // 데이터 패킷 송신
    return TWI_TransCheck_ACK(MT_DATA_ACK);
}

// STOP
static inline void TWI_Stop(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

// RESTART
static inline uint8_t TWI_Restart(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    return TWI_TransCheck_ACK(TWI_RESTART);
}

// Master transmit one byte
static inline uint8_t TWI_Master_Transmit(uint8_t Data, uint8_t Addr)
{
    uint8_t ret = TWI_Start();
    if (ret) return ret;
    ret = TWI_Write_SLAW(Addr);
    if (ret) return ret;
    ret = TWI_Write_Data(Data);
    if (ret) return ret;
    TWI_Stop();
    return 0;
}

// SLA+R
static inline uint8_t TWI_Write_SLAR(uint8_t Addr)
{
    TWDR = (uint8_t)(Addr | 0x01);       // set R bit
    TWCR = (1<<TWINT) | (1<<TWEN);
    return TWI_TransCheck_ACK(MR_SLA_ACK);
}

// DATA (read with ACK)
static inline uint8_t TWI_Read_Data(uint8_t* Data)
{
    uint8_t ret;
    TWCR = (1<<TWINT) | (1<<TWEN);
    ret = TWI_TransCheck_ACK(MR_DATA_ACK);
    if (ret) return ret;
    *Data = TWDR;
    return 0;
}

// DATA (read with NACK)
static inline uint8_t TWI_Read_Data_NACK(uint8_t* Data)
{
    uint8_t ret;
    TWCR = (1u<<TWINT) | (1u<<TWEN);
    ret = TWI_TransCheck_ACK(MR_DATA_NACK);
    if (ret) return ret;
    *Data = TWDR;
    return 0;
}

// Master receive one byte
static inline uint8_t TWI_Master_Receive(uint8_t Addr, uint8_t* Data)
{
    uint8_t ret, rec;
    ret = TWI_Start();             if (ret) return ret;
    ret = TWI_Write_SLAR(Addr);    if (ret) return ret;
    ret = TWI_Read_Data(&rec);     if (ret) return ret;
    TWI_Stop();
    *Data = rec;
    return 0;
}

// ---- Slave (Receiver) helpers ----
static inline void Init_TWI_Slaveaddr(uint8_t Slave_Addr)
{
    TWAR = Slave_Addr;
}

static inline uint8_t TWI_Slave_Match_ACK(void)
{
    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    return TWI_TransCheck_ACK(SR_SLA_ACK);
}

static inline uint8_t TWI_Slave_Stop_ACK(void)
{
    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    return TWI_TransCheck_ACK(SR_STOP);
}

static inline uint8_t TWI_Slave_Read_Data(uint8_t* Data)
{
    uint8_t ret;
    TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
    ret = TWI_TransCheck_ACK(SR_DATA_ACK);
    if (ret) return ret;
    *Data = TWDR;
    return 0;
}

static inline uint8_t TWI_Slave_Receive(uint8_t* Data)
{
    uint8_t ret, rec;
    ret = TWI_Slave_Match_ACK();   if (ret) return ret;
    ret = TWI_Slave_Read_Data(&rec); if (ret) return ret;
    ret = TWI_Slave_Stop_ACK();    if (ret) return ret;
    *Data = rec;					// 수신 데이터 반환
    return 0;
}

// Mixed (e.g., EEPROM register read)
static inline uint8_t TWI_Master_Receive_ExDevice(uint8_t devAddr, uint8_t regAddr, uint8_t* Data)
{
    uint8_t ret, rec;
    ret = TWI_Start();                 if (ret) return ret;
    ret = TWI_Write_SLAW(devAddr);     if (ret) return ret;
    ret = TWI_Write_Data(regAddr);     if (ret) return ret;
    ret = TWI_Restart();               if (ret) return ret;
    ret = TWI_Write_SLAR(devAddr);     if (ret) return ret;
    ret = TWI_Read_Data_NACK(&rec);    if (ret) return ret;
    TWI_Stop();
    *Data = rec;
    return 0;
}


// ================= SIMPLE MODE =================
#else  // !_USE_SAFTY_TWI_

// Return 0 on success, 1 on mismatch (preserve original simple behavior)
static inline uint8_t TWI_TransCheck_ACK(uint8_t Stat)
{
    // Poll until TWINT is set
    while (!(TWCR & (1u<<TWINT))) { /* wait */ }
    // Check status
    if ((TWSR & 0xF8) != Stat) return 1;
    return 0;
}

// START (void)
static inline void TWI_Start(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(TWI_START);
}

// SLA+W (void)
static inline void TWI_Write_SLAW(uint8_t Addr)
{
    TWDR = Addr;
    TWCR = (1<<TWINT) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(MT_SLA_ACK);
}

// DATA write (void)
static inline void TWI_Write_Data(uint8_t Data)
{
    TWDR = Data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(MT_DATA_ACK);
}

// STOP
static inline void TWI_Stop(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
}

// RESTART (void)
static inline void TWI_Restart(void)
{
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(TWI_RESTART);
}

// Master transmit (void)
static inline void TWI_Master_Transmit(uint8_t Data, uint8_t Addr)
{
    TWI_Start();
    TWI_Write_SLAW(Addr);
    TWI_Write_Data(Data);
    TWI_Stop();
}

// SLA+R (void)
static inline void TWI_Write_SLAR(uint8_t Addr)
{
    TWDR = (uint8_t)(Addr | 0x01u);
    TWCR = (1<<TWINT) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(MR_SLA_ACK);
}

// DATA read (with ACK) -> return byte
static inline uint8_t TWI_Read_Data(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(MR_DATA_ACK);
    return TWDR;
}

// DATA read (with NACK) -> return byte
static inline uint8_t TWI_Read_Data_NACK(void)
{
    TWCR = (1<<TWINT) | (1<<TWEN);
    (void)TWI_TransCheck_ACK(MR_DATA_NACK);
    return TWDR;
}

// Master receive one byte
static inline uint8_t TWI_Master_Receive(uint8_t Addr)
{
    uint8_t rec;
    TWI_Start();
    TWI_Write_SLAR(Addr);
    rec = TWI_Read_Data();
    TWI_Stop();
    return rec;
}

// ---- Slave (Receiver) helpers ----
static inline void Init_TWI_Slaveaddr(uint8_t Slave_Addr)
{
    TWAR = Slave_Addr;
}

static inline void TWI_Slave_Match_ACK(void)
{
    TWCR = (1<<TWINT) | (1<<TWEA) | (1u<<TWEN);
    (void)TWI_TransCheck_ACK(SR_SLA_ACK);
}

static inline void TWI_Slave_Stop_ACK(void)
{
    TWCR = (1<<TWINT) | (1<<TWEA) | (1u<<TWEN);
    (void)TWI_TransCheck_ACK(SR_STOP);
}

static inline uint8_t TWI_Slave_Read_Data(void)
{
    TWCR = (1<<TWINT) | (1<<TWEA) | (1u<<TWEN);
    (void)TWI_TransCheck_ACK(SR_DATA_ACK);
    return TWDR;
}

static inline uint8_t TWI_Slave_Receive(void)
{
    uint8_t data;
    TWI_Slave_Match_ACK();
    data = TWI_Slave_Read_Data();
    TWI_Slave_Stop_ACK();
    return data;
}

#endif // _USE_SAFTY_TWI_

#endif // _INCLUDE_TWI_H__
