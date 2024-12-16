/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) 2022, OHORONNYI HOLDING. All rights reserved.                                                  */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
//***********************************************************************************************************
//  File Function: N76E003 UART RS485 slave demo code
//***********************************************************************************************************
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "stdio.h"

#include "rf/if.h"
#include "rf/hw_layer.h"
#include "rf/cmt2300a.h"



#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(b) (1UL << (b))




//project's definitions
#define RXTOUT 100  //Timeout after RX byte for end of data 10 mS (value in x100uS)
#define SENDTOUT 1393 //Timeout for sending packets in test mode

#define MAXDATA 30  //maximal payload length (correspond CMT2300 buffer size!)

unsigned char tx_buf[MAXDATA+3]; //UART Tx buffer for output Radio received packets
volatile unsigned char tx_ptr=0; //pointer to byte will be outputted

unsigned char rx_buf[MAXDATA+3]; //UART RX buffer
volatile unsigned char rx_ptr=0; //pointer to byte will be received over UART

unsigned char tmp[MAXDATA+3]; //temporary buffer

unsigned char ison=0; //flag of Radio CMT2300A is connected
unsigned char send_cnt=0; //counter of sended packets in test mode
unsigned char test,Request,ID_dev=0,hi_hex,low_hex,num_last_low,num_last_hi;
unsigned int acc_rssi=0;



//------------------------Timer's managment-------------------------

//start unblocked sending timeout interval (value in 1 mS, maximum 1394mS)
void set_send_tout(unsigned int tout){
     clr_TR2;  //clr_T2CON_TR2;              //Stop Timer2
     clr_TF2;  //clr_T2CON_TF2;              //clear interupt flag
     if(tout){ //set timeout and start timer
          tout=tout * 47;           //value for clk=24MHz and div=512
          tout = tout - (tout>>10); //fine tune to 1000uS
          tout = tout - (tout>>9);  
          tout = ~tout;  //convert to counter init value
         
          TL2 = (0x0F&tout);  //LOBYTE(tout); //set counter's startup value
          TH2 = ((0xF0&tout)>>4);  //HIBYTE(tout);
          set_TR2; //set_T2CON_TR2;  //start Timer 2
     }	
}


//start unblocked timeout interval (value in 100us)
void set_uart_tout(unsigned char tout){  //1 tick is 0.5 uS 
     unsigned short us;
     if(!tout) us=65535; else us=~((unsigned short)tout*200); //period is 65536-us*200
     TR1=0; //clr_TCON_TR1; //stop timer 1
     TF1=0; //clr_TCON_TF1;	//clear timeout flag
     TL1 = (0x0F&us);  //TL1 = LOBYTE(us); //set low byte of period
     TH1 = ((0xF0&us)>>4);  //TH1 = HIBYTE(us); //set hight byte of period
     TR1=1;	//set_TCON_TR1;	//start timer		
}

void Send_Hex(char dec_symbol){
     unsigned char high,low;
     high=((dec_symbol>>0x04) & 0x0F) + 0x30;      // + '0'
     if (high>0x39) high +=7;                             //>'9'
     hi_hex=high;
     // low nibble
     low=(dec_symbol & 0x0F) + 0x30;               // +'0' low nibble
     if (low>0x39) low +=7;
     low_hex=low;
     return;
}
//-------------------------Interupts--------------------------------
 //UART0 interupt
 void SerialPort0_ISR(void) interrupt 4 
{
   	// _push_(SFRS);
  if (RI){  //RX data enable
    clr_RI; // Clear RI (Receive Interrupt).
    rx_buf[rx_ptr++]=SBUF; //put receved byte to RX buffer
    rx_ptr&=31; //restrict data length is 31 byte
    if (rx_ptr<30) set_uart_tout(RXTOUT); //clear UART timeout
    else {  //we have 30 bytes
       cmt_set_cts(0); //or set CTS while rx buffer full
       set_uart_tout(0); //force timeout
       //after CTS=0 only one extra byte can be received into rx_buf[30]
       //in this case rx_ptr will be 31
    }
  }    
  if (TI){   //TX compleet 
    clr_TI;  // Clear TI (Transmit Interrupt).
    if(tx_ptr>0) SBUF = tx_buf[tx_ptr--]; //check there is unsended bytes, send next, move pointer		
  }
     // _pop_(SFRS);
   
}
//-------------------------------------------------------
void initMCU(void){
//-----------------GPIO Setup--------------------------
	//Outputs
     //OnBoard LED
     P03_PushPull_Mode;
     P03=1;
     //TestOut - JAMPER 2
     P14=0;
     P14_PushPull_Mode;
     //SPI DIO - was P01
     P01=1;
     //P01_PUSHPULL_MODE;
     P01_Quasi_Mode;
     //SPI SCK - was P10
     P10=1;
     P10_PushPull_Mode;
     //SPI CS
     P12=1;
     P12_PushPull_Mode;
     //SPI FB - was P22
     P11=1;
     P11_PushPull_Mode;
     //UART CTS - was P33
     //P33=1;
     //P33_PUSHPULL_MODE;
     //SET 2 color led tx_status - was P24
     P30=1;    //red
     P30_PushPull_Mode;
     P17=1;//green - was P13
     P17_PushPull_Mode;
     //Inputs with pull-up
     //IN 3; - was P14
     P05=1;
     P05_Input_Mode;
     P05_PushPull_Mode;
     //UART RTS - was P03
     //P03=1;
     //P03_QUASI_MODE;
     //TestIn (Jumper for activate sending test) - was P17 - BUTTON
     P02=1;
     P02_Input_Mode;
     P02_PushPull_Mode;	
     // ID dev 2 jumpers for select id device(0..3) - was P21
     P16=1;
     P16_Input_Mode;
     P16_PushPull_Mode;
     
     P14=1;
     P14_Input_Mode;
     P14_PushPull_Mode;
     if (!P14&&!P16) ID_dev=0;	else
     if (!P14&&P16)  ID_dev=1;	else
     if (P14&&!P16)  ID_dev=2;	else
     if (P14&&P16)   ID_dev=3;
//--------------------------------------------
	//setup timer0 for blocking delay in Radio driver
     clr_T0M;                                  //T0M=0, Timer0 Clock = Fsys/12
     TMOD |= 0x01;                                   //Timer0 is 16-bit mode
	//setup timer 1 for unblocking UART timeout
     clr_T1M;                                  //T1M=0, Timer1 Clock = Fsys/12
     TMOD |= 0x10;                                   //Timer01 is 16-bit mode	
	// setup timer2 for unblocking sending timeout
	clr_TR2;              //Stop Timer2
     clr_TF2;              //clear interupt flag
     T2MOD&=0x8F;
     TIMER2_DIV_512; //T2MOD|=0x70;  //Timer2 Clock = Fsys/512
     TIMER2_Auto_Reload_Delay_Mode; //clr_CMRL2;        //Timer 2 as auto-reload mode
     set_LDEN;
     set_CMPCR;       //Timer 2 value is auto-cleared as 0000H when a compare match occurs.  
	//setup UART
     //MODIFY_HIRC(HIRC_24);
     P06_PushPull_Mode;
     P07_Input_Mode;
     
     set_ES;  //enable UART0 RX and TX interupts
     set_EA; //enable interupts globally


}
//--------------------------------------------

void main(void){
     unsigned char b,TX_status,RX_test; //temporary value
	RX_test=0;
	//start:	
     initMCU();
     InitialUART0_Timer1(115200);  //UART_Open(24000000,UART0_Timer3,115200);  //run uart with boudrate generator is Timer3 
     printf("Start ");
    //start Radio
	Radio_Init();  //initialize CMT2300A radio
	ison=Radio_On(1); //start receiving with testing device exist	
     if (ison){
        Send_Data_To_UART0('O'); Send_Data_To_UART0('K'); Send_Data_To_UART0(0x13); Send_Data_To_UART0(0x10);
        //	CMT2300A_SetFrequencyChannel(200);
     //	CMT2300A_SetNodeID(0);
	   als1();  //indicate Radio started OK   
     } else {
        Send_Data_To_UART0('F'); Send_Data_To_UART0('a'); Send_Data_To_UART0('i'); Send_Data_To_UART0('l'); Send_Data_To_UART0(0x13); Send_Data_To_UART0(0x10);
     }
 
    
     set_send_tout(SENDTOUT); //run autosend timer
	cmt_set_cts(1); //set flag UART is ready
     printf("Ready Radio \n");
	//main loop
	while(1){
		//---------------------check having packet for transmitt-----------------------			
          //check some data readed and combytetimeout
		if (P02==0) {
               test=1;
               Request=0;
          } else {
               test=0;
               Request=1;
          }
          if (rx_ptr && TF1&&!test){ 
               cmt_set_cts(0); //deny reading uart
               for (b=0;b<rx_ptr;b++) tmp[b]=rx_buf[b]; //copy readed data to temporary buffer			 
			 //normally rx_ptr is 1-30: a number of bytes for send
			 //rx_ptr can be maximum 31: one extra byte can be received after CTS=0  
               if (rx_ptr==31){ //special case: one extra byte received
				b--; //decrese length of packet will be transmitted: 30 bytes is maximum
				rx_buf[0]=rx_buf[30]; //copy extra byte as first byte of new packet
				rx_ptr=1; //set pointer: one byte already received
				set_uart_tout(RXTOUT); //clear UART timeout					 
			 } else rx_ptr=0; //1-30 bytes were received: all will be sended
			 cmt_set_cts(1); //allow reading uart
			 if(ison) {
                     Radio_Send_VarLen(tmp, b);
                } //transmitt data (blocked), j is data length
          }
		//--------------------check receive packet------------------------------		
          //check for receive packet by CMT2300		 
          if (!test||RX_test)	b=Radio_Recv_VarLen(tmp);	//b is length of receved packet
          if (b && (!tx_ptr)&&(!test||RX_test)){ //check data received and tx buffer is free
               unsigned char i1=0;	 
               tx_ptr=b; //set pointer to first byte will be output over UART	 	
               if (Request&&tmp[8]=='R'&&tmp[9]=='s'&&tmp[10]=='s'){
                    Send_Hex((char)CMT2300A_GetRssiDBm());
                    tmp[16]=hi_hex;
                    tmp[17]=low_hex;
               }
               while(b) {
                    tx_buf[b--]=tmp[i1++];
               }	//P03=1;copy received bytes in reverse order    
               Send_Data_To_UART0(tx_buf[tx_ptr--]); //start TX first byte over UART0	
               if (RX_test&&tmp[0]=='O'&&tmp[1]=='K'&&tmp[3]==num_last_hi&&tmp[4]==num_last_low){
                    P17=1;
                    if (!P17) set_P30; else clr_P30;
				RX_test=0;
               }
               if (Request&&tmp[3]=='P'&&tmp[4]=='C'){
                    tmp[3]=tmp[0];tmp[4]=tmp[1];
                    tmp[0]='O';tmp[1]='K';
                    tmp[2]=' ';	 
                    P03 ^= 1;
                    if (ison) Radio_Send_VarLen(tmp, 5);
               }
          }
	 //---------------------check for test sendigng mode--------------------
          if (test&&(TF2)){ //check test mode jumper is close and send timeout occured
               unsigned char i2=0;
               if(RX_test)RX_test=0;
               //acc_rssi+=CMT2300A_GetRssiCode();
               acc_rssi=CMT2300A_GetRssiDBm();
               //acc_rssi/=2;
               Send_Hex(send_cnt);
               tmp[0]=hi_hex;
               tmp[1]=low_hex;
               num_last_hi=tmp[0];
               num_last_low=tmp[1];
               tmp[2]=' ';
               tmp[3]='P';tmp[4]='C';tmp[5]='K';tmp[6]='T';tmp[7]=' ';tmp[8]='R';
               tmp[9]='s';tmp[10]='s';tmp[11]='I';tmp[12]=' ';
               Send_Hex((char)acc_rssi);
               tmp[13]=hi_hex; tmp[14]=low_hex;
               tmp[15]=' ';tmp[16]=' ';tmp[17]=' ';tmp[18]=' ';
               tmp[19]='I';tmp[20]='D';tmp[21]=' ';tmp[22]=0x30+ID_dev;
               tmp[23]=10; tmp[24]=13;
               if (ison) TX_status=Radio_Send_VarLen(tmp, 25);  //check radio avaliable and send 1 byte of counter
               P03 ^= 1; //blink LED on each sended packet in test mode
               if (TX_status) {
                    P17=0;
                    clr_P30; 
			// for(i2=0;i2<20;i2++) CMT2300A_DelayUs(1000);
                    RX_test=1;
               } else {
                    P17=0;
                    if (!P17) set_P30; else clr_P30;
               }
               send_cnt++; //count sended packets			 
               set_send_tout(SENDTOUT); //restart autosend timer for next sending
          } 
	}
}



