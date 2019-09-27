

#include "types.h"
#include "state.h"
#include "i2c.h"
#include "motor.h"
#include "stop.h"

volatile int dummy = 0; // used for reading register and ignoring value

// state data for bipolar motor
uint8  stateByte;
int16  targetPos;
uint16 targetSpeed;
bool   targetDir;
bool   noBounds;
int16  curPos;
uint16 curSpeed;
bool   curDir;
int16  backlashPos; // neg is left of dead zone, >= backlashWid is right
uint8  ustep;
uint16 acceleration;
bool   stepPending;
bool   stepped;
bool   stopping;
bool   homing;
uint8  homingState;
bool   slowing;
uint8  phase;  // bipolar: matches phase inside drv8825, unipolar: step phase
uint16 nextStepTicks;
uint16 lastStepTicks;
bool   haveCommand;
bool   resetAfterSoftStop;
bool   nextStateSpecialVal; // return homeTestPos or lim sw on next read
int16  homeTestPos;         // pos when limit sw closes
bool   haveLimit;

void setStateBit(uint8 mask, uint8 set){
  disableAllInts;
  stateByte = (stateByte & ~mask) | (set ? mask : 0);
  enableAllInts;
}

void setError(uint8 err) {
  if(err == CLEAR_ERROR) {
    disableAllInts;
    stateByte = stateByte & ~ERR_CODE;
    enableAllInts;
    dummy = I2C_BUF_BYTE;   // clear SSPOV
  }
  else {
    stateByte = err;
    resetMotor();
  }
}

volatile uint8 errorIntCode;

// used in interrupt
void setErrorInt(uint8 err) {
  errorIntCode = err;
}
