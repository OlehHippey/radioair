#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "stdio.h"
#include "gpiofunc.h"

void ledRedGreen(char ledStatus){
  if (ledStatus==LED_OFF){
      LED_PIN_GREEN=0; LED_PIN_RED=0;
  } else if (ledStatus==LED_GREEN) {
       LED_PIN_GREEN=1;   LED_PIN_RED=0;
  } else if (ledStatus==LED_RED) {
       LED_PIN_GREEN=0;   LED_PIN_RED=1;
  } else if (ledStatus==LED_TOGGLE) {
       LED_PIN_GREEN=~LED_GREEN;   LED_PIN_RED=!LED_PIN_GREEN;
  } else {
        LED_PIN_GREEN=1;   LED_PIN_RED=1;
  }
}