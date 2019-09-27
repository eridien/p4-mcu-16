
#include <xc.h>
#include "pins.h"
#include "state.h"
#include "stop.h"
#include "motor.h"
#include "move.h"
#include "clock.h"

void stopStepping() {
  mState->stepPending = false;
  mState->stepped     = false;
  mState->homing      = false;
  mState->slowing     = false;
  mState->stopping    = false;
  mState->curSpeed    = 0;
  setStateBit(BUSY_BIT, 0);
}

void resetMotor() {
  resetLAT = 0;
  stopStepping();
  setStateBit(MOTOR_ON_BIT, 0);
  setStateBit(HOMED_BIT, 0);
}

void softStopCommand(bool resetAfter) {
  mState->slowing            = true;
  mState->homing             = false;
  mState->targetDir          = mState->curDir;
  mState->targetSpeed        = 0;
  mState->resetAfterSoftStop = resetAfter;
  if((mState->stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    mState->lastStepTicks = timeTicks;
    enableAllInts;
    mState->curSpeed = mSet.val.jerk; // triggers shutdown code
  }
  setStateBit(BUSY_BIT, 1);
  mState->stopping = true;
}

void chkStopping() {
  if(mState->curSpeed <= mSet.val.jerk || mSet.val.accelIdx == 0) {
    stopStepping();
    if(mState->resetAfterSoftStop) {
      resetMotor();
    }
    return;
  }
}



