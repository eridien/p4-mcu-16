
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
  switch(homingState) {
    case movingToFwdSide:
      if(!limitSwOn()) {
        // start normal homing
        targetDir = mSet.val.homingDir;
        homingState = goingHome;
      }
      break;
            
    case goingHome:
      if(limitSwOn()) {
        // arrived at switch while homing backward
        // HOMED_BIT not cleared until now, homing might have been interrupted
        setStateBit(HOMED_BIT, false);
        targetDir   = !mSet.val.homingDir;
        targetSpeed = mSet.val.homingBackUpSpeed;
        homingState = homeReversing;
      }
      break;
      
    case homeReversing:
      if(!limitSwOn()) {
        // passed switch second (or third) time
        homeTestPos = curPos;
        curPos      = 0;
        homingState = homingToOfs;
       }
      break;
    
    case homingToOfs: 
      if(mSet.val.homingDir ? (curPos <= mSet.val.homeOfs) 
                       : (curPos >= mSet.val.homeOfs)) {
        homing = false;
        setStateBit(HOMED_BIT, 1);
        curPos = mSet.val.homePos;
        stopStepping();
        return;
      }
      break;
  }
}
void homeCommand(bool start) {
  slowing = false;
  if((stateByte & BUSY_BIT) == 0) {
    // not moving -- init speed
    motorOn();
    disableAllInts;
    lastStepTicks = timeTicks;
    enableAllInts;
    curSpeed = mSet.val.jerk;
  }
  if(start && haveLimit) {
    homing = true;
    if(limitSwOn()) {
      // go to fwd side of switch at full homing speed
      homingState = movingToFwdSide;
      targetDir   = !mSet.val.homingDir;
    }
    else {
      homingState = goingHome;
      targetDir   = mSet.val.homingDir;
    }
    targetSpeed = mSet.val.homingSpeed;
    setStateBit(BUSY_BIT,  1);
    // homed sState bit unchanged until we get to limit switch
    //   so homing can be interrupted by move command
  }
  else {
    // fake homing for motors with no limit switch
    // hard stop with no reset
    // set wherever it lands to home pos
    stopStepping();
    curPos = mSet.val.homePos;
    setStateBit(HOMED_BIT, 1);
  }
}
