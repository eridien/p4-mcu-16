#ifndef I2C_H
#define	I2C_H

#include <xc.h>
#include "types.h"
#include "motor.h"

#define RECV_BUF_SIZE   (NUM_SETTING_WORDS*2 + 1) // + opcode byte
#define NUM_SEND_BYTES   3  //  state, posH, posL

#define I2C_START_BIT SSP1STATbits.S
#define I2C_STOP_BIT  SSP1STATbits.P
#define I2C_BUF_BYTE  SSP1BUF
#define I2C_SSPIF     SSP1IF
#define RdNotWrite    SSP1STATbits.I2C_READ
#define NotAddr       SSP1STATbits.NOT_ADDRESS
#define NotStretch    SSP1CON1bits.CKP
#define I2C_SSPOV     SSP1CON1bits.SSPOV

// motor is bottom 2 bits in addr
// addr is set based on ID input pins
extern uint8 mcuLoc;
#define LOC_BASE 0
#define LOC_X    1
#define LOC_HEAD 2

#define MCU_BASE (mcuLoc == LOC_BASE)
#define MCU_X    (mcuLoc == LOC_X   )
#define MCU_HEAD (mcuLoc == LOC_HEAD)

#define I2C_ADDR_MASK 0xf8 // motor idx in d2-d1 (d1-d0 in real addr)

extern uint8 i2cAddrBase; 
#define I2C_ADDR_BASE   0x18  // real addr: 0x14+motor (base mcu)
#define I2C_ADDR_X      0x20  // real addr: 0x18+motor (   X mcu)
#define I2C_ADDR_HEAD   0x28  // real addr: 0x1c+motor (head mcu)

extern volatile uint8 i2cRecvBytes[RECV_BUF_SIZE + 1];
extern volatile uint8 i2cRecvBytesPtr;
extern volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
extern volatile uint8 i2cSendBytesPtr;

void setMcuLoc(void);
void i2cInit(void);
void i2cInterrupt(void);

#endif	/* I2C_H */
