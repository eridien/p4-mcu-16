
#include <xc.h>
#include "pins.h"
#include "state.h"
#include "stop.h"
#include "motor.h"
#include "move.h"
#include "clock.h"

void stopStepping() {
  stepPending = false;
  stepped     = false;
  homing      = false;
  slowing     = false;
  stopping    = false;
  curSpeed    = 0;
  setStateBit(BUSY_BIT, 0);
}

void resetMotor() {
  resetLAT = 0;
  stopStepping();
  setStateBit(MOTOR_ON_BIT, 0);
  setStateBit(HOMED_BIT, 0);
}

void softStopCommand(bool resetAfter) {
  slowing            = true;
  homing             = false;
  targetDir          = curDir;
  targetSpeed        = 0;
  resetAfterSoftStop = resetAfter;
  if((stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    lastStepTicks = timeTicks;
    enableAllInts;
    curSpeed = mSet.val.jerk; // triggers shutdown code
  }
  setStateBit(BUSY_BIT, 1);
  stopping = true;
}

void chkStopping() {
  if(curSpeed <= mSet.val.jerk || mSet.val.accelIdx == 0) {
    stopStepping();
    if(resetAfterSoftStop) {
      resetMotor();
    }
    return;
  }
}



