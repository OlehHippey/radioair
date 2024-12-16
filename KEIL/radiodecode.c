#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "stdio.h"
#include "radiodecode.h"

extern unsigned char rx_buf[MAXDATA+3];
extern volatile unsigned char rx_ptr; //pointer to byte will be received over UART

void clearRxBuff(){
  char i=0;
  for (i=0;i<MAXDATA+3; i++) rx_buf[i]=0;
  rx_ptr=0;
}

void clearRxBuffISR(){
  char i=0;
  for (i=0;i<MAXDATA+3; i++) rx_buf[i]=0;
  rx_ptr=0;   
}