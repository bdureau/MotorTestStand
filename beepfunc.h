#ifndef _BEEPFUNC_H
#define _BEEPFUNC_H
#include "config.h"
#include "Arduino.h"
#if defined TESTSTANDESP32 || defined TESTSTANDESP32V3
#include <ESP32Tone.h>
#endif
extern boolean noContinuity;
extern boolean NoBeep;
extern const int pinSpeaker;
extern int beepingFrequency;

extern void beginBeepSeq();
extern void longBeep();
extern void shortBeep();
extern void beepTestStandVersion (int majorNbr, int minorNbr);
#endif
