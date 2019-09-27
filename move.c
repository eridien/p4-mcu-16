
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
  
  if(mState->homing) {
    if (mSet.val.accelIdx == 0 || 
        mState->curSpeed <= mSet.val.jerk) {
      mState->curSpeed = mState->targetSpeed;
      mState->curDir   = mState->targetDir;
    }
    else {
      decelerate = true;
    }
  }
  else if(mState->stopping) {
    decelerate = true;
  }
  else {
    // normal move to target position

    int16 distRemaining = (mState->targetPos - mState->curPos);
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
        mState->curSpeed = mState->targetSpeed;
        mState->curDir   = mState->targetDir = (mState->targetPos > mState->curPos);
      }
      else {
        // using acceleration
        if (mState->curSpeed <= mSet.val.jerk) {
          // going slower than accel threshold
          
          if(mState->curPos == mState->targetPos) {
            // finished normal move
            stopStepping();
            return;
          }
          // can chg dir any time when slow
          mState->curDir = mState->targetDir = distRemPositive;
        }
        
        // going faster than accel threshold
        else if(mState->slowing) {
          decelerate = true;
        }
        else {
          if(mState->curDir != mState->targetDir) {
            // we need to chg dir but we are going too fast
            decelerate = true;
          }
          else {
            // look up decel dist target
            uint16 distTgt = calcDist(mSet.val.accelIdx, mState->curSpeed);
            if(distRemaining < distTgt) {
              decelerate = true;
              mState->slowing = true;
            }
          }
        }
        if(!decelerate && !accelerate && !closing) {
          if(mState->curSpeed > mState->targetSpeed) {
            decelerate = true;
          }
          else if(!mState->slowing && mState->curSpeed < mState->targetSpeed) {
            accelerate = true;
          }
        }
      }
    }
  }
  if(decelerate) {
    // accel/step = accel/sec / steps/sec
    uint16 deltaSpeed = ((uint32) mState->acceleration * 8) / mState->curSpeed;
    if(deltaSpeed == 0) deltaSpeed = 1;
    if(mState->curSpeed >= deltaSpeed) {
      mState->curSpeed -= deltaSpeed;
    } 
    else {
      mState->curSpeed = mSet.val.jerk;
    }
  }
  else if (accelerate) {
    // accel/step = accel/sec / steps/sec
   uint16 deltaSpeed = ((uint32) mState->acceleration * 8) / mState->curSpeed;
   if(deltaSpeed == 0) deltaSpeed = 1;
    mState->curSpeed += deltaSpeed;
    if(mState->curSpeed > mState->targetSpeed) {
      // we just passed target speed
      // we should never go faster than target speed
      mState->curSpeed = mState->targetSpeed;
    }
  }
  uint16 clkTicks;
  if(!closing) {
    // adjust ustep
    uint8 tgtUstep;
    // we want pps to be between 750 and 1500, if possible
    // low pps gives sw more time to keep up
    if      (mState->curSpeed > (8192 + 4096) / 2) tgtUstep = 0;
    else if (mState->curSpeed > (4096 + 2048) / 2) tgtUstep = 1;
    else if (mState->curSpeed > (2048 + 1024) / 2) tgtUstep = 2;
    else                                       tgtUstep = 3;

    if(tgtUstep != mState->ustep) {
      // you can only change ustep when the drv8825 phase is correct
      if((mState->phase & uStepPhaseMask[tgtUstep]) == 0) {
        mState->ustep = tgtUstep;
      }
    }
  }
  else { 
  // we are closing in (very close to target))
    // if needed, adjust ustep to hit exact position
    // phase is always correct since only decreasing step size
    int8 dist = mState->targetPos - mState->curPos;
    if(dist < 0) dist = -dist;
    while(mState->ustep < 3 && uStepDist[mState->ustep] > dist) {
      mState->ustep++;
    }
  }
  if(mState->ustep > mSet.val.maxUstep) {
     mState->ustep = mSet.val.maxUstep;
  }
  // set step timing
  switch (mState->ustep) {
    case 0:  clkTicks = clkTicksPerSec / (mState->curSpeed >> 3); break;
    case 1:  clkTicks = clkTicksPerSec / (mState->curSpeed >> 2); break;
    case 2:  clkTicks = clkTicksPerSec / (mState->curSpeed >> 1); break;
    case 3:  clkTicks = clkTicksPerSec /  mState->curSpeed      ; break;
    default: clkTicks = 0; // to avoid compiler warning
  }

  bool err;
  disableAllInts;
  mState->nextStepTicks = mState->lastStepTicks + clkTicks;
  // modulo 2**16 arithmetic
  err = (mState->nextStepTicks - (timeTicks+1)) > 32000;
  enableAllInts;
  mState->stepped = false;
  if(err) { 
    // nextStepTicks is in the past
    setError(STEP_NOT_DONE_ERROR); 
  } else {
    stepLAT = 0;
    mState->stepPending = true;
  }
}

void moveCommand(bool noRules) {
  mState->noBounds = noRules;
  
  if((mState->stateByte & HOMED_BIT) == 0 && !noRules) {
    setError(NOT_HOMED);
    return;
  }
  mState->slowing     = false;
  mState->homing      = false;
  mState->stopping    = false;
  mState->targetDir   = (mState->targetPos >= mState->curPos);   
  if(mState->curSpeed == 0 || (mState->stateByte & BUSY_BIT) == 0) {
    disableAllInts;
    mState->lastStepTicks = timeTicks;
    enableAllInts;
    mState->curSpeed = mSet.val.jerk;
    mState->curDir   = mState->targetDir;
  }
  setStateBit(BUSY_BIT, 1);
}



