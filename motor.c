
#include <xc.h>
#include "types.h"
#include "pins.h"
#include "motor.h"
#include "i2c.h"
#include "state.h"
#include "clock.h"
#include "home.h"
#include "move.h"
#include "stop.h"

bool haveSettings;
union settingsUnion mSet;

void motorInit() {
  dirTRIS = 0;
  ms1TRIS = 0;
  ms2TRIS = 0;
  
  resetLAT  = 0; // start with reset on
  resetTRIS = 0;  
  
  stepLAT   = 1; 
  stepTRIS  = 0;

  faultTRIS = 1; // zero input means motor fault
  limTRIS   = 1; // limit switch input

  haveSettings = false;
  stateByte = 0; // no err, not busy, motor off, and not homed
  phase = 0; // cur step phase
  haveCommand = false;
  stepPending = false;
  stepped = false;
  curSpeed = 0;
}
#include "i2c.h" // DEBUG

bool haveFault() {
  return !faultLAT;
}

bool limitSwOn() {
  return (haveLimit && (limLAT == 0));
}

// setting words are big endian
// write may be short, only setting first entries

void setMotorSettings(uint8 numWordsRecvd) {
  uint8 i;
  for (i = 0; i < numWordsRecvd; i++) {
    mSet.reg[i] = (i2cRecvBytes[2 * i + 2] << 8) |
                             i2cRecvBytes[2 * i + 3];
  }
  acceleration = accelTable[mSet.val.accelIdx];
  setTicksSec();
  clkTicksPerSec = ((uint16) (1000000 / mSet.val.mcuClock));
  haveSettings = true;
}

// from event loop

void checkAll() {
  if (haveFault()) {
    setError(MOTOR_FAULT_ERROR);
    return;
  }
  if (stepPending) {
    return;
  }
  if (stepped) {
    stepped = false;
    uint8 stepDist = uStepDist[ustep];
    int8  signedDist = ((curDir) ? stepDist : -stepDist); 
    phase += signedDist;
    
    if(mSet.val.backlashWid) {
      if((backlashPos < 0) && curDir) {
        // reversing from backward to forward outside dead zone
        backlashPos = stepDist;
        signedDist -= mSet.val.backlashWid;
        if(signedDist < 0) signedDist = 0;
      }
      else if((backlashPos >= (int16) mSet.val.backlashWid) && !curDir) {
        // reversing from forward to backward outside dead zone
        backlashPos = mSet.val.backlashWid - stepDist;
        signedDist += mSet.val.backlashWid;
        if(signedDist > 0) signedDist = 0;
      }
      else if(backlashPos >= 0 && backlashPos < (int16) mSet.val.backlashWid){
        // moving inside backlash dead zone
        backlashPos += signedDist;
        if(backlashPos < 0) {
          signedDist = backlashPos;
        }
        else if(backlashPos >= (int16) mSet.val.backlashWid) {
          signedDist = backlashPos - mSet.val.backlashWid;
        }
        else signedDist = 0;
      }
    }
    curPos += signedDist;
  }
  if ((stateByte & BUSY_BIT) && !haveError()) {
    if (homing) {
      chkHoming();
    } else if (stopping) {
      chkStopping();
    } else {
      // normal moving
      if ((curPos < mSet.val.minPos || curPos > mSet.val.maxPos) 
           && !noBounds) {
        setError(BOUNDS_ERROR);
        return;
      }
    }
  }
  if ((stateByte & BUSY_BIT) && !haveError()) {
    checkMotor();
  }
}

void motorOn() {
  setStateBit(MOTOR_ON_BIT, 1);
  if (resetLAT == 0) {
    resetLAT = 1;
    // counter in drv8825 is cleared by reset
    // ms->phase always matches internal phase counter in drv8825
    phase = 0;
  }
}

uint8 numBytesRecvd;

// called on every command except settings
bool lenIs(uint8 expected, bool chkSettings) {
  if(chkSettings && !haveSettings) {
    setError(NO_SETTINGS);
    return false;
  }
  if (expected != numBytesRecvd) {
    setError(CMD_DATA_ERROR);
    return false;
  }
  return true;
}

void processCommand() {
  volatile uint8 *rb = ((volatile uint8 *) i2cRecvBytes);
  numBytesRecvd   = rb[0];
  uint8 firstByte = rb[1];
  if ((firstByte & 0x80) == 0x80) {
    if (lenIs(2, true)) {
      // move command
      targetSpeed = mSet.val.speed;
      targetPos = ((int16) (firstByte & 0x7f) << 8) | rb[2];
      moveCommand(false);
    }
  } else if ((firstByte & 0xc0) == 0x40) {
    // speed-move command
    if (lenIs(3, true)) {
      // changes settings for speed
      mSet.val.speed = (uint16) (firstByte & 0x3f) << 8;
      targetSpeed = mSet.val.speed;
      targetPos = (int16) (((uint16) rb[2] << 8) | rb[3]);
      moveCommand(false);
    }
  } else if ((firstByte & 0xf8) == 0x08) {
    // accel-speed-move command
    if (lenIs(5, true)) {
      // changes settings for acceleration and speed
      mSet.val.accelIdx = (firstByte & 0x07);
      mSet.val.speed = (((uint16) rb[2] << 8) | rb[3]);
      acceleration = accelTable[mSet.val.accelIdx];
      targetSpeed = mSet.val.speed;
      targetPos = (int16) (((uint16) rb[4] << 8) | rb[5]);
      moveCommand(false);
    }
  } else if ((firstByte & 0xe0) == 0x20) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(2, true)) {
      motorOn();
      uint16 dist = ((( (uint16) firstByte & 0x0f) << 8) | rb[2]);
      // direction bit is in d4
      if(firstByte & 0x10) targetPos = curPos + dist;
      else                 targetPos = curPos - dist;
      acceleration = 0;
      targetSpeed  = mSet.val.jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x02) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(3, true)) {
      motorOn(); 
      targetPos    = curPos + (int16) (((uint16) rb[2] << 8) | rb[3]);
      acceleration = 0;
      targetSpeed  = mSet.val.jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x03) {
    // jog command relative - no bounds checking and doesn't need to be homed
    if (lenIs(3, true)) {
      motorOn();
      targetPos    = (int16) (((uint16) rb[2] << 8) | rb[3]);
      acceleration = 0;
      targetSpeed  = mSet.val.jerk;
      moveCommand(true);
    }
  } else if (firstByte == 0x01) {
    // setPos command
    if (lenIs(3, false)) {
      curPos =  (int16) (((uint16) rb[2] << 8) | rb[3]);
    }
  } else if (firstByte == 0x1f) {
    // load settings command
    uint8 numWords = (numBytesRecvd - 1) / 2;
    if ((numBytesRecvd & 0x01) == 1 &&
            numWords > 0 && numWords <= NUM_SETTING_WORDS) {
      setMotorSettings(numWords);
    } else {
      setError(CMD_DATA_ERROR);
    }
  } else if((firstByte & 0xfc) == 0x04) {
   // next status contains special value
    if (lenIs(1, true)) {
      nextStateSpecialVal = (firstByte & 0x03) + 1;
    } else {
      setError(CMD_DATA_ERROR);
    }
  } else if ((firstByte & 0xf0) == 0x10) {

    uint8 bottomNib = firstByte & 0x0f;
    // one-byte commands
    if (lenIs(1, (bottomNib != 4 && bottomNib != 7))) {
      switch (bottomNib) {
        case 0: homeCommand(true);           break; // start homing
        case 2: softStopCommand(false);      break; // stop,no reset
        case 3: softStopCommand(true);       break; // stop with reset
        case 4: resetMotor();                break; // hard stop (immediate reset)
        case 5: motorOn();                   break; // reset off
        case 6: homeCommand(false);          break; // stop, set curpos to setting
        default: setError(CMD_DATA_ERROR);
      }
    }
  } 
  else setError(CMD_DATA_ERROR);
}
void __attribute__((interrupt, shadow, auto_psv)) _T1Interrupt(void) {
  _T1IF = 0;
  timeTicks++;
  if (stepPending && nextStepTicks == timeTicks) {
    if (stepped) {
      // last motor step not handled yet
      setErrorInt(STEP_NOT_DONE_ERROR);
      return;
    }
    ms1LAT = ((ustep & 0x01) ? 1 : 0);
    ms2LAT = ((ustep & 0x02) ? 1 : 0);
    dirLAT =   curDir        ? 1 : 0;
    stepLAT = 1;
    stepPending = false;
    lastStepTicks = timeTicks;
    stepped = true;
  }
}
