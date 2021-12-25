//================================================================
// beeping functions
//================================================================
#include "beepfunc.h"

boolean noContinuity = false;


boolean NoBeep = false;

#ifdef TESTSTAND
const int pinSpeaker = PB4; //12;
#endif

#ifdef TESTSTANDSTM32
const int pinSpeaker = PA0;
#endif

#ifdef TESTSTANDSTM32V2
const int pinSpeaker = PA0;
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
      tone(pinSpeaker, 1600, 1000);
      delay(50);
      noTone(pinSpeaker);
    }
    delay(1000);
  }
}
void longBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, beepingFrequency, 1000);
    delay(1500);
    noTone(pinSpeaker);
  }
}
void shortBeep()
{
  if (NoBeep == false)
  {
    tone(pinSpeaker, beepingFrequency, 25);
    delay(300);
    noTone(pinSpeaker);
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
