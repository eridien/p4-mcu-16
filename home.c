
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "state.h"
#include "home.h"
#include "motor.h"
#include "stop.h"
#include "move.h"
#include "clock.h"

void chkHoming() {
  switch(mState->homingState) {
    case movingToFwdSide:
      if(!limitSwOn()) {
        // start normal homing
        mState->targetDir = mSet.val.homingDir;
        mState->homingState = goingHome;
      }
      break;
            
    case goingHome:
      if(limitSwOn()) {
        // arrived at switch while homing backward
        // HOMED_BIT not cleared until now, homing might have been interrupted
        setStateBit(HOMED_BIT, false);
        mState->targetDir   = !mSet.val.homingDir;
        mState->targetSpeed = mSet.val.homingBackUpSpeed;
        mState->homingState = homeReversing;
      }
      break;
      
    case homeReversing:
      if(!limitSwOn()) {
        // passed switch second (or third) time
        mState->homeTestPos = mState->curPos;
        mState->curPos      = 0;
        mState->homingState = homingToOfs;
       }
      break;
    
    case homingToOfs: 
      if(mSet.val.homingDir ? (mState->curPos <= mSet.val.homeOfs) 
                       : (mState->curPos >= mSet.val.homeOfs)) {
        mState->homing = false;
        setStateBit(HOMED_BIT, 1);
        mState->curPos = mSet.val.homePos;
        stopStepping();
        return;
      }
      break;
  }
}
void homeCommand(bool start) {
  mState->slowing = false;
  if((mState->stateByte & BUSY_BIT) == 0) {
    // not moving -- init speed
    motorOn();
    disableAllInts;
    mState->lastStepTicks = timeTicks;
    enableAllInts;
    mState->curSpeed = mSet.val.jerk;
  }
  if(start && mState->haveLimit) {
    mState->homing = true;
    if(limitSwOn()) {
      // go to fwd side of switch at full homing speed
      mState->homingState = movingToFwdSide;
      mState->targetDir   = !mSet.val.homingDir;
    }
    else {
      mState->homingState = goingHome;
      mState->targetDir   = mSet.val.homingDir;
    }
    mState->targetSpeed = mSet.val.homingSpeed;
    setStateBit(BUSY_BIT,  1);
    // homed sState bit unchanged until we get to limit switch
    //   so homing can be interrupted by move command
  }
  else {
    // fake homing for motors with no limit switch
    // hard stop with no reset
    // set wherever it lands to home pos
    stopStepping();
    mState->curPos = mSet.val.homePos;
    setStateBit(HOMED_BIT, 1);
  }
}
