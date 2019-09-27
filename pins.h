  
#ifndef PINS_H
#define	PINS_H

#define IDTRIS0    _TRISA0   // mcu ID bit 0 (clr on X mcu)
#define IDPORT0    _RA0      // ms1, only valid at startup
#define IDTRIS1    _TRISA1   // mcu ID bit 1 (clr on HEAD mcu)
#define IDPORT1    _RA1      // ms2, only valid at startup

#define ms1TRIS   _TRISA0
#define ms2TRIS   _TRISA1
#define dirTRIS   _TRISA2
#define stepTRIS  _TRISA3
#define resetTRIS _TRISA4
#define faultTRIS _TRISA6
#define limTRIS   _TRISB2

#define ms1LAT    _LATA0
#define ms2LAT    _LATA1
#define dirLAT    _LATA2
#define stepLAT   _LATA3
#define resetLAT  _LATA4
#define faultLAT  _LATA6
#define limLAT    _LATB2

#endif	/* PINS_H */

