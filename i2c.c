
#include <xc.h>
#include "types.h"
#include "state.h"
#include "pins.h"
#include "i2c.h"
#include "state.h"
#include "motor.h"

volatile uint8 i2cRecvBytes[RECV_BUF_SIZE+1]; // added len byte
volatile uint8 i2cRecvBytesPtr;
volatile uint8 i2cSendBytes[NUM_SEND_BYTES];
volatile uint8 i2cSendBytesPtr;
volatile bool  inPacket;
volatile bool  packetForUs;

uint8 i2cAddrBase; 
uint8 mcuLoc;

// must be run before RA0/RA1 tristate turned off
void setMcuLoc(void) {
  IDTRIS0  = 1;            // MCU ID 0 (ms1))
  IDTRIS1  = 1;            // MCU ID 1 (ms2))
  if(IDPORT0 == 0) {
    mcuLoc = LOC_X;
    i2cAddrBase = I2C_ADDR_X;
  } else if(IDPORT1 == 0) {
    mcuLoc = LOC_HEAD;
    i2cAddrBase = I2C_ADDR_HEAD;
  } else {
    mcuLoc = LOC_BASE;
    i2cAddrBase = I2C_ADDR_BASE;
  }
}

void i2cInit() { 
  SSP1CON1bits.SSPM = 0x0e;          // slave mode, 7-bit, S & P ints enabled 
  SSP1MSK           = I2C_ADDR_MASK; // address mask, check top 5 bits
  SSP1ADD           = i2cAddrBase;   // slave address (7-bit addr << 1)
  SSP1STATbits.SMP  = 0;             // slew-rate enabled
  SSP1STATbits.CKE  = 1;             // smb voltage levels
  SSP1CON2bits.SEN  = 1;             // enable clock stretching 
  SSP1CON3bits.AHEN = 0;             // no clock stretch before addr ack
  SSP1CON3bits.DHEN = 0;             // no clock stretch before data ack
  SSP1CON3bits.BOEN = 1;             // enable buffer overwrite check
  NotStretch        = 0;             // stretch clk of first start bit
  
  _SSP1IF = 0;                       // nothing received yet
  _SSP1IE = 1;                       // Enable ints
  
  SSP1CON1bits.SSPEN = 1;            // Enable the serial port
}

// all words are big-endian
void setSendBytesInt() {
  switch (nextStateSpecialVal) {
    case 0:
      i2cSendBytes[0] = (MCU_VERSION | stateByte);
      i2cSendBytes[1] =  curPos >> 8;
      i2cSendBytes[2] =  curPos & 0x00ff;   
      break;
    case 1: 
      i2cSendBytes[0] = (MCU_VERSION | AUX_RES_BIT | 0);
      i2cSendBytes[1] = homeTestPos >> 8;
      i2cSendBytes[2] = homeTestPos & 0x00ff;
      break;        
    case 2: 
      i2cSendBytes[0] = (MCU_VERSION | AUX_RES_BIT | 1);
      i2cSendBytes[1] = 0; 
      i2cSendBytes[2] = limLAT;
      break;      
    default: 
      setErrorInt(CMD_DATA_ERROR);
  }
  nextStateSpecialVal = 0;
}

volatile uint8 motIdxInPacket;

void __attribute__ ((interrupt,shadow,auto_psv)) _MSSP1Interrupt(void) {
  _SSP1IF = 0;
  
  // SSPxSTATbits.S is set during entire packet
  if(I2C_START_BIT && !inPacket) { 
    // received start bit, prepare for packet
    i2cRecvBytesPtr = 1;    // skip over length byte
    i2cSendBytesPtr = 1;    // first is hard-wired to zero
    I2C_SSPOV       = 0;    // clear SSPOV
    inPacket     = true;
    packetForUs  = false;
  }
  else if(I2C_STOP_BIT) { 
    // received stop bit
    inPacket = false;
    if(packetForUs) {
      if (I2C_SSPOV) {
        setErrorInt(OVERFLOW_ERROR);
        I2C_SSPOV = 0;   // clear SSPOV
      }
      else {
        if(!RdNotWrite) {
          // done receiving -- total length of recv is stored in first byte
          i2cRecvBytes[0] = i2cRecvBytesPtr-1;
          // tell event loop that data is available
          haveCommand = true;
        } else {
          // sent last byte of status packet
          if(i2cSendBytes[0] & ERR_CODE) {
            // master just read status with error code, clear it
            setErrorInt(CLEAR_ERROR);
          }
        }
      }
    }
  }
  else {
    if(!NotAddr) { 
      // received addr byte, extract motor number and read bit
      packetForUs    = true;
      motIdxInPacket = (I2C_BUF_BYTE & 0x06) >> 1;
      if(RdNotWrite) {
        // prepare all send data
        setSendBytesInt(motIdxInPacket);
        // send packet (i2c read from slave), load buffer for first byte
        I2C_BUF_BYTE = i2cSendBytes[0];
      }
    }
    else {
      if(!RdNotWrite) {
        // received byte (i2c write to slave)
        if (haveCommand != 0) {
            // last command for this motor not handled yet by event loop
            setErrorInt(OVERFLOW_ERROR);
        }
        else {
          if(i2cRecvBytesPtr < RECV_BUF_SIZE + 1) 
            i2cRecvBytes[i2cRecvBytesPtr++] = I2C_BUF_BYTE;
        }
      }
      else {
        // sent byte (i2c read from slave), load buffer for next send
        I2C_BUF_BYTE = i2cSendBytes[i2cSendBytesPtr++];
      }
    }
  }
  // in packet: set ckp to end stretch after ack
  // stop bit:  clr ckp so next start bit will stretch
  NotStretch = !I2C_STOP_BIT; 
}
