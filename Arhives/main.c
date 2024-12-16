/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: ML51 simple GPIO toggle out demo code
//***********************************************************************************************************

#include "MS51_32K.h"
#include "rf/if.h"
#include "rf/hw_layer.h"
#include "rf/cmt2300a.h"
/**
 * @brief       CMT2300W demo
 * @param       None
 * @return      None
 * @details     UART <-> CMT2300A radio <-> CMT2300A radio <-> UART
 */

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
void set_send_tout(unsigned int tout)
{
 clr_T2CON_TR2;              //Stop Timer2
 clr_T2CON_TF2;              //clear interupt flag
 if(tout) //set timeout and start timer
 {
  tout=tout * 47;           //value for clk=24MHz and div=512
  tout = tout - (tout>>10); //fine tune to 1000uS
  tout = tout - (tout>>9);  
  tout = ~tout;  //convert to counter init value
	 
  TL2 = LOBYTE(tout); //set counter's startup value
  TH2 = HIBYTE(tout);
  set_T2CON_TR2;  //start Timer 2
 }	
}


//start unblocked timeout interval (value in 100us)
void set_uart_tout(unsigned char tout)
{  //1 tick is 0.5 uS 
 unsigned short us;
 if(!tout) us=65535;
 else us=~((unsigned short)tout*200); //period is 65536-us*200
 clr_TCON_TR1; //stop timer 1
 clr_TCON_TF1;	//clear timeout flag
 TL1 = LOBYTE(us); //set low byte of period
 TH1 = HIBYTE(us); //set hight byte of period
 set_TCON_TR1;	//start timer		
}
void Send_Hex(char dec_symbol)
{
  unsigned char high,low;
  high=((dec_symbol>>0x04) & 0x0F) + 0x30;      // + '0'
  if (high>0x39)
    high +=7;                             //>'9'
  hi_hex=high;
  // low nibble
  low=(dec_symbol & 0x0F) + 0x30;               // +'0' low nibble
  if (low>0x39)
    low +=7;
  low_hex=low;
  return;
}

//-------------------------Interupts--------------------------------

 //UART0 interupt

 void Serial_ISR (void) interrupt 4 
 {
	 _push_(SFRS);
    if (RI)  //RX data enable
    {   	
			clr_SCON_RI; // Clear RI (Receive Interrupt).
			rx_buf[rx_ptr++]=SBUF; //put receved byte to RX buffer
      rx_ptr&=31; //restrict data length is 31 byte
			if(rx_ptr<30) set_uart_tout(RXTOUT); //clear UART timeout
			else  //we have 30 bytes
			{  
				cmt_set_cts(0); //or set CTS while rx buffer full
				set_uart_tout(0); //force timeout
				//after CTS=0 only one extra byte can be received into rx_buf[30]
				 //in this case rx_ptr will be 31
			}
					
    }
    if (TI)  //TX compleet
    {       
        clr_SCON_TI;  // Clear TI (Transmit Interrupt).
			  if(tx_ptr>0) SBUF = tx_buf[tx_ptr--]; //check there is unsended bytes, send next, move pointer		
    }
	 
	 _pop_(SFRS);
 }
 
 //WakeUp timer Interupt
/*void WakeUp_Timer_ISR (void)   interrupt 17     //ISR for self wake-up timer
{
      _push_(SFRS);
   clr_WKCON_WKTF;  //clear interrupt flag   
      _pop_(SFRS);
}
*/

//--------------------------------main procedure-------------------------
void main (void) 
{
  unsigned char b,TX_status,RX_test; //temporary value
	RX_test=0;
	//start:	
	//-----------------GPIO Setup--------------------------
	
	//Outputs
	
	   //OnBoard LED 
	   P12_PUSHPULL_MODE;
	   P12=1;
	   //TestOut
	   P35=0;
	   P35_PUSHPULL_MODE;
	
	   //SPI DIO
	   P01=1;
	   //P01_PUSHPULL_MODE;
	   P01_QUASI_MODE;
	   
	   //SPI SCK
	   P10=1;
		 P10_PUSHPULL_MODE;
	   //SPI CS
     set_P2_3;
	   P23_PUSHPULL_MODE;
	   //SPI FB
		 set_P2_2;
	   P22_PUSHPULL_MODE;
		 //UART CTS
		 P33=1;
	   P33_PUSHPULL_MODE;
		 //SET 2 color led tx_status
		 set_P2_4;    //red
	   P24_PUSHPULL_MODE;
		 P13=1;//green
	   P13_PUSHPULL_MODE;
		 //Inputs with pull-up
		 //IN 3;
	   P14=1;
	   P14_INPUT_MODE;
		 ENABLE_P14_PULLUP;
	   //UART RTS
		 P03=1;
	   P03_QUASI_MODE;
     		 
	   //TestIn (Jumper for activate sending test)
		 P17=1;
	  P17_INPUT_MODE;
		 ENABLE_P17_PULLUP;	
// ID dev 2 jumpers for select id device(0..3)
		 set_P2_1;
	  P21_INPUT_MODE;
		 ENABLE_P21_PULLUP;			 
		 P35=1;
	  P35_INPUT_MODE;
		 ENABLE_P35_PULLUP;
   if (!P35&&!(P2&0x02)) ID_dev=0;	else
   if (!P35&&(P2&0x02))  ID_dev=1;	else
	 if (P35&&!(P2&0x02))  ID_dev=2;	else
	 if (P35&&(P2&0x02))   ID_dev=3;	
	 
//--------------------------------------------	
	//setup timer0 for blocking delay in Radio driver
	  clr_CKCON_T0M;                                  //T0M=0, Timer0 Clock = Fsys/12
    TMOD |= 0x01;                                   //Timer0 is 16-bit mode
	
	//setup timer 1 for unblocking UART timeout
	  clr_CKCON_T1M;                                  //T1M=0, Timer1 Clock = Fsys/12
    TMOD |= 0x10;                                   //Timer01 is 16-bit mode
	
	// setup timer2 for unblocking sending timeout
	clr_T2CON_TR2;              //Stop Timer2
  clr_T2CON_TF2;              //clear interupt flag
  T2MOD&=0x8F;T2MOD|=0x70;  //Timer2 Clock = Fsys/512
  clr_T2CON_CMRL2;        //Timer 2 as auto-reload mode
  set_T2MOD_LDEN;
  set_T2MOD_CMPCR;       //Timer 2 value is auto-cleared as 0000H when a compare match occurs.
		
	//setup UART
	  MODIFY_HIRC(HIRC_24);
    P06_PUSHPULL_MODE;
    P07_INPUT_MODE;
    UART_Open(24000000,UART0_Timer3,115200);  //run uart with boudrate generator is Timer3 
		//UART_Send_Data(UART0, 0x31);
		ENABLE_UART0_INTERRUPT;  //enable UART0 RX and TX interupts
    ENABLE_GLOBAL_INTERRUPT; //enable interupts globally

  //start Radio
	Radio_Init();  //initialize CMT2300A radio
	
	ison=Radio_On(1); //start receiving with testing device exist	
	UART_Send_Data(UART0, ison);
//	CMT2300A_SetFrequencyChannel(200);
//	CMT2300A_SetNodeID(0);
	if(ison) als1();  //indicate Radio started OK
  set_send_tout(SENDTOUT); //run autosend timer
	cmt_set_cts(1); //set flag UART is ready
	
	//main loop
	while(1)
	{
		//---------------------check having packet for transmitt-----------------------			
			//check some data readed and combytetimeout
		if(P17==0) {test=1;Request=0;} else {test=0;Request=1;}
	if(rx_ptr && TF1&&!test)//
			{ 
        cmt_set_cts(0); //deny reading uart				
			 for(b=0;b<rx_ptr;b++) tmp[b]=rx_buf[b]; //copy readed data to temporary buffer
			 
			 //normally rx_ptr is 1-30: a number of bytes for send
			 //rx_ptr can be maximum 31: one extra byte can be received after CTS=0  
				if(rx_ptr==31) //special case: one extra byte received
			 {
				b--; //decrese length of packet will be transmitted: 30 bytes is maximum
				rx_buf[0]=rx_buf[30]; //copy extra byte as first byte of new packet
				rx_ptr=1; //set pointer: one byte already received
				set_uart_tout(RXTOUT); //clear UART timeout					 
			 }
			 else rx_ptr=0; //1-30 bytes were received: all will be sended
			 cmt_set_cts(1); //allow reading uart
			 
			 if(ison) {Radio_Send_VarLen(tmp, b);} //transmitt data (blocked), j is data length
			 
			}
				
			//--------------------check receive packet------------------------------		
  //check for receive packet by CMT2300		 
	if(!test||RX_test)	b=Radio_Recv_VarLen(tmp);	//b is length of receved packet
  //  if (TF2)	
	//	{	
	//		UART_Send_Data(UART0,b);
	//		set_send_tout(SENDTOUT);
	//	}	
	 if(b && (!tx_ptr)&&(!test||RX_test)) //check data received and tx buffer is free
   { 
	   unsigned char i1=0;	 
     tx_ptr=b; //set pointer to first byte will be output over UART	 	
		 if (Request&&tmp[8]=='R'&&tmp[9]=='s'&&tmp[10]=='s'){
			 Send_Hex((char)CMT2300A_GetRssiDBm());
			 tmp[16]=hi_hex;
			 tmp[17]=low_hex;
		 }
		 while(b) {tx_buf[b--]=tmp[i1++];}	//P12=1;copy received bytes in reverse order    
			 UART_Send_Data(UART0,tx_buf[tx_ptr--]	); //start TX first byte over UART0	
     if (RX_test&&tmp[0]=='O'&&tmp[1]=='K'&&tmp[3]==num_last_hi&&tmp[4]==num_last_low)
        {P13=1;
					if(!P13)set_P2_4;else clr_P2_4;
				RX_test=0;}
		 if (Request&&tmp[3]=='P'&&tmp[4]=='C'){
			 tmp[3]=tmp[0];tmp[4]=tmp[1];
		  tmp[0]='O';tmp[1]='K';
			 tmp[2]=' ';
			 
			 P12 ^= 1;
	//		tx_buf[1]= 'K';tx_buf[2]='O';
			// tx_ptr=3;
			// UART_Send_Data(UART0,tx_buf[tx_ptr--]);
			// for(i1=0;i1<20;i1++) CMT2300A_DelayUs(1000);
			 if(ison)Radio_Send_VarLen(tmp, 5);
			 //for(i1=0;i1<20;i1++) CMT2300A_DelayUs(1000); 
		 }
		 // P12 ^= 1;//blink LED on each received packet
		 
	 }
	 
	 //---------------------check for test sendigng mode--------------------
	 if(test&&(TF2)) //check test mode jumper is close and send timeout occured
	 {
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
		 if(ison) TX_status=Radio_Send_VarLen(tmp, 25);  //check radio avaliable and send 1 byte of counter
		 P12 ^= 1; //blink LED on each sended packet in test mode
		 if (TX_status) {P13=0;clr_P2_4; 
			// for(i2=0;i2<20;i2++) CMT2300A_DelayUs(1000);
		   RX_test=1;
		 }else  {P13=0;if(!P13)set_P2_4;else clr_P2_4;}
		 send_cnt++; //count sended packets			 
		 set_send_tout(SENDTOUT); //restart autosend timer for next sending
		 
	 } 
	 
	 //-------------------------check for CTS signal entering sleep-----------------------	 
	 //read control from Master, go to POWER DOWN mode
	/* if(!cmt_read_rts())
		{
			//entering power down mode:			
		 //disable UART	
		 cmt_set_cts(0); //set CTS signal for master indicated sleep mode
			
     DISABLE_UART0_INTERRUPT; //disable UART0 RX and TX interupts
     P07_QUASI_MODE;  //set UART pins as GPIO                           
     P06_QUASI_MODE; 		
				
		//sleep Radio	
		 Radio_On(0); //put CMT2300 to sleep mode  
  
     //set all GPIO as inputs
     ALL_GPIO_INPUT_MODE;
		
		//set special function for on-board LED, CTS and RTS	
    //LED
    P12=1;
    P12_PUSHPULL_MODE;

    //UART CTS output
    P03=0;
    P03_PUSHPULL_MODE;

    //UART RTS input
    P33=1;
    P33_QUASI_MODE;	

// Modify HIRC to 24MHz for UART baud rate function only 
   MODIFY_HIRC(HIRC_24);
    //manage clock
		ClockEnable(FSYS_LIRC);
		FsysSelect(FSYS_LIRC); //select 38.4KHz internal clock
    ClockDisable(FSYS_HIRC); //disable 24MHz internal clock
    
		//start wake-up timer with 1 sec tick
	//	WKT_Open (FSYS_LIRC,256,150); //Open wake-up timer 38400Hz / 256 = 150Hz -> 1sec
	  SFRS = 0;
    WKCON = 0x02;                     //timer base 10k, Pre-scale = 1/16
    SFRS = 2;
    RWKH = 0xFD;
    SFRS = 0;
    RWKL = 0X8F;                      //  if prescale is 0x00, never set RWK = 0xff
    ENABLE_WKT_INTERRUPT;                          // enable WKT interrupt
    set_WKCON_WKTR;                         // Wake-up timer run 
    set_EIE1_EWKT;    // Enable WKT interrupt
    ENABLE_GLOBAL_INTERRUPT;  // Enable Global interrupt
    DISABLE_BOD; //disable voltage fail dedtect for decrese current
    als0();	//of on-board LED 		
		//put MCU power down mode with periodically wake by timer and poll RTS
		 //wake every 1 sec and check RTS pin level
		 while(! cmt_read_rts()) //check RTS pin level in loop
     {
			 set_PCON_PD; //1 sec power down while RTS=0 
			 P12 ^= 1; //1 sec blink in sleep mode (test only!)
		 }    
		 //exiting power down mode	 
     ENABLE_BOD; //enable brownout detect
		 clr_WKCON_WKTR;//stop wake-up timer
//manage clock
     FsysSelect(FSYS_HIRC); //select 24MHz system clock
     ClockDisable(FSYS_LIRC); //disable 34.8KHz clock
//restart all
    	tx_ptr=0; //clear global values
      rx_ptr=0;
      ison=0;
		  send_cnt=0;
      goto start; //restart firmware for entering work mode		 
		}	
*/
	}

}
