
#ifndef STATE_H
#define	STATE_H

#include "types.h"
#include "motor.h"

#define MCU_VERSION 0

extern volatile int dummy;

#define disableAllInts __builtin_disi(0x3FFF)
#define enableAllInts  __builtin_disi(0x0000) 

// Error codes 
#define MOTOR_FAULT_ERROR   0x10
#define OVERFLOW_ERROR      0x20
#define CMD_DATA_ERROR      0x30
#define STEP_NOT_DONE_ERROR 0x40
#define BOUNDS_ERROR        0x50
#define NO_SETTINGS         0x60
#define NOT_HOMED           0x70
#define CLEAR_ERROR         0xff // magic code to clear error

// state byte
#define ERR_CODE            0x70
#define AUX_RES_BIT         0x08 // do-d1 indicate what is in pos word
#define BUSY_BIT            0x04
#define MOTOR_ON_BIT        0x02
#define HOMED_BIT           0x01

extern  uint8  stateByte;
extern  int16  targetPos;
extern  uint16 targetSpeed;
extern  bool   targetDir;
extern  bool   noBounds;
extern  int16  curPos;
extern  uint16 curSpeed;
extern  bool   curDir;
extern  int16  backlashPos; // neg is left of dead zone, >= backlashWid is right
extern  uint8  ustep;
extern  uint16 acceleration;
extern  bool   stepPending;
extern  bool   stepped;
extern  bool   stopping;
extern  bool   homing;
extern  uint8  homingState;
extern  bool   slowing;
extern  uint8  phase;  // bipolar: matches phase inside drv8825, unipolar: step phase
extern  uint16 nextStepTicks;
extern  uint16 lastStepTicks;
extern  bool   haveCommand;
extern  bool   resetAfterSoftStop;
extern  bool   nextStateSpecialVal; // return homeTestPos or lim sw on next read
extern  int16  homeTestPos;         // pos when limit sw closes
extern  bool   haveLimit;

#define haveError() (errorIntCode || (stateByte & ERR_CODE))

extern volatile uint8 errorIntMot;
extern volatile uint8 errorIntCode;

void  setStateBit(uint8 mask, uint8 set);
void  setError(uint8 err);
void  setErrorInt(uint8 err);
void  clrErrorInt(void);

#endif	/* STATE_H */

