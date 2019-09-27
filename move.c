
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "move.h"
#include "state.h"
#include "motor.h"
#include "clock.h"
#include "stop.h"
#include "dist-table.h"
#include "debug.h"

const uint16 uStepPhaseMask[4] = {0x07, 0x03, 0x01, 0x00};
const uint16 uStepDist[4]      = {   8,    4,    2,    1};

//  accel is 0..7: none, 4000, 8000, 20000, 40000, 80000, 200000, 400000 steps/sec/sec
//  for 1/40 mm steps: none, 100, 200, 500, 1000, 2000, 5000, 10000 mm/sec/sec
const uint16 accelTable[8] = // (steps/sec/sec accel) / 8
       {0, 500, 1000, 2500, 5000, 10000, 25000, 50000};

void checkMotor() {
  bool  accelerate = false;
  bool  decelerate = false;
  bool  closing    = false;
  
  if(homing) {
    if (mSet.val.accelIdx == 0 || 
        curSpeed <= mSet.val.jerk) {
      curSpeed = targetSpeed;
      curDir   = targetDir;
    }
    else {
      decelerate = true;
    }
  }
  else if(stopping) {
    decelerate = true;
  }
  else {
    // normal move to target position

    int16 distRemaining = (targetPos - curPos);
    bool  distRemPositive = (distRemaining >= 0);
    if(!distRemPositive) {
      distRemaining = -distRemaining;
    }
    if(distRemaining == 0) {
      // finished normal move
      stopStepping();
      return;
    }
    if(distRemaining <= uStepDist[MIN_USTEP]) {
      // dist is smaller than largest step
      // adjust ustep each pulse to make sure to hit target exactly
      closing = true;
    }
    
    
    else {
      if (mSet.val.accelIdx == 0) {
        // not using acceleration
        curSpeed = targetSpeed;
        curDir   = targetDir = (targetPos > curPos);
      }
      else {
        // using acceleration
        if (curSpeed <= mSet.val.jerk) {
          // going slower than accel threshold
          
          if(curPos == targetPos) {
            // finished normal move
            stopStepping();
            return;
          }
          // can chg dir any time when slow
          curDir = targetDir = distRemPositive;
        }
        
        // going faster than accel threshold
        else if(slowing) {
          decelerate = true;
        }
        else {
          if(curDir != targetDir) {
            // we need to chg dir but we are going too fast
            decelerate = true;
          }
          else {
            // look up decel dist target
            uint16 distTgt = calcDist(mSet.val.accelIdx, curSpeed);
            if(distRemaining < distTgt) {
              decelerate = true;
              slowing = true;
            }
          }
        }
        if(!decelerate && !accelerate && !closing) {
          if(curSpeed > targetSpeed) {
            decelerate = true;
          }
          else if(!slowing && curSpeed < targetSpeed) {
            accelerate = true;
          }
        }
      }
    }
  }
  if(decelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = ((uint32) acceleration * 8) / curSpeed;
    if(deltaSpeed == 0) deltaSpeed = 1;
    if(curSpeed >= deltaSpeed) {
      curSpeed -= deltaSpeed;
    } 
    else {
      curSpeed = mSet.val.jerk;
    }
  }
  else if (accelerate) {
    // accel/step = accel/sec / steps/sec
   uint16 deltaSpeed = ((uint32) acceleration * 8) / curSpeed;
   if(deltaSpeed == 0) deltaSpeed = 1;
    curSpeed += deltaSpeed;
    if(curSpeed > targetSpeed) {
      // we just passed target speed
      // we should never go faster than target speed
      curSpeed = targetSpeed;
    }
  }
  uint16 clkTicks;
  if(!closing) {
    // adjust ustep
    uint8 tgtUstep;
    // we want pps to be between 750 and 1500, if possible
    // low pps gives sw more time to keep up
    if      (curSpeed > (8192 + 4096) / 2) tgtUstep = 0;
    else if (curSpeed > (4096 + 2048) / 2) tgtUstep = 1;
    else if (curSpeed > (2048 + 1024) / 2) tgtUstep = 2;
    else                                       tgtUstep = 3;

    if(tgtUstep != ustep) {
      // you can only change ustep when the drv8825 phase is correct
      if((phase & uStepPhaseMask[tgtUstep]) == 0) {
        ustep = tgtUstep;
      }
    }
  }
  else { 
  // we are closing in (very close to target))
    // if needed, adjust ustep to hit exact position
    // phase is always correct since only decreasing step size
    int8 dist = targetPos - curPos;
    if(dist < 0) dist = -dist;
    while(ustep < 3 && uStepDist[ustep] > dist) {
      ustep++;
    }
  }
  if(ustep > mSet.val.maxUstep) {
     ustep = mSet.val.maxUstep;
  }
  // set step timing
  switch (ustep) {
    case 0:  clkTicks = clkTicksPerSec / (curSpeed >> 3); break;
    case 1:  clkTicks = clkTicksPerSec / (curSpeed >> 2); break;
    case 2:  clkTicks = clkTicksPerSec / (curSpeed >> 1); break;
    case 3:  clkTicks = clkTicksPerSec /  curSpeed      ; break;
    default: clkTicks = 0; // to avoid compiler warning
  }

  bool err;
  disableAllInts;
  nextStepTicks = lastStepTicks + clkTicks;
  // modulo 2**16 arithmetic
  err = (nextStepTicks - (timeTicks+1)) > 32000;
  enableAllInts;
  stepped = false;
  if(err) { 
    // nextStepTicks is in the past
    setError(STEP_NOT_DONE_ERROR); 
  } else {
    stepLAT = 0;
    stepPending = true;
  }
}

void moveCommand(bool noRules) {
  noBounds = noRules;
  
  if((stateByte & HOMED_BIT) == 0 && !noRules) {
    setError(NOT_HOMED);
    return;
  }
  slowing     = false;
  homing      = false;
  stopping    = false;
  targetDir   = (targetPos >= curPos);   
  if(curSpeed == 0 || (stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    lastStepTicks = timeTicks;
    enableAllInts;
    curSpeed = mSet.val.jerk;
    curDir   = targetDir;
  }
  setStateBit(BUSY_BIT, 1);
}



