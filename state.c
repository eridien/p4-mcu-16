

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"
#include "stop.h"

volatile int dummy = 0; // used for reading register and ignoring value

motorState  mStateS;
motorState *mState = &mStateS;

void setStateBit(uint8 mask, uint8 set){
  disableAllInts;
  mState->stateByte = (mState->stateByte & ~mask) | (set ? mask : 0);
  enableAllInts;
}

void setError(uint8 err) {
  if(err == CLEAR_ERROR) {
    disableAllInts;
    mState->stateByte = mState->stateByte & ~ERR_CODE;
    enableAllInts;
    dummy = I2C_BUF_BYTE;   // clear SSPOV
  }
  else {
    mState->stateByte = err;
    resetMotor();
  }
}

volatile uint8 errorIntCode;

// used in interrupt
void setErrorInt(uint8 err) {
  errorIntCode = err;
}
