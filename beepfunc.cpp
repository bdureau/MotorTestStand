//================================================================
// beeping functions
//================================================================
#include "beepfunc.h"

boolean noContinuity = false;


boolean NoBeep = false;

#ifdef TESTSTAND
const int pinSpeaker = PB4; //12;
#endif

#if defined TESTSTANDSTM32 || defined TESTSTANDSTM32V2
const int pinSpeaker = PA0;
#endif

/*#ifdef TESTSTANDSTM32V2
const int pinSpeaker = PA0;
#endif*/

#ifdef TESTSTANDESP32
const int pinSpeaker = 4;
#endif
int beepingFrequency =440;

/*int lastPin = -1;
int currentPinPos = 0;
int currentPin = -1;
int pos = -1;*/

long tempo = 0;
long startTempo = 0;
long tempo2 = 0;
long startTempo2 = 0 ;
long bigdelay = 0;
long savedDelay =0;



void beginBeepSeq()
{
  int i = 0;
  if (NoBeep == false)
  {
    for (i = 0; i < 10; i++)
    {
      #ifdef TESTSTANDESP32
      tone(pinSpeaker, 1600, 50);
      noTone(pinSpeaker);
      delay(50);
      #endif
      #ifndef TESTSTANDESP32
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
      #endif
    }
    delay(1000);
  }
}
void longBeep()
{
  if (NoBeep == false)
  {
    #ifdef TESTSTANDESP32
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1000);
    noTone(pinSpeaker);
    #endif
    #ifndef TESTSTANDESP32
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1500);
    noTone(pinSpeaker);
    #endif
  }
}
void shortBeep()
{
  if (NoBeep == false)
  {
    #ifdef TESTSTANDESP32
    tone(pinSpeaker, beepingFrequency, 25);
    noTone(pinSpeaker);
    delay(300);
    #endif
    #ifndef TESTSTANDESP32
    tone(pinSpeaker, beepingFrequency, 25);
    delay(300);
    noTone(pinSpeaker);
    #endif
  }
}
void beepTestStandVersion (int majorNbr, int minorNbr)
{
  int i;
  for (i = 0; i < majorNbr; i++)
  {
    longBeep();
  }
  for (i = 0; i < minorNbr; i++)
  {
    shortBeep();
  }
}
