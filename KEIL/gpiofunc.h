
//GPIO names
#define PIN_DATA P00
#define PIN_TEST P04
#define LED_STATUS P03
#define LED_STATUS_SET P03=1
#define LED_STATUS_OFF P03=0

#define LED_PIN_GREEN P30
#define LED_PIN_RED   P17

//led functions 
#define LED_OFF    0  
#define LED_GREEN  1  
#define LED_RED    2  
#define LED_TOGGLE 3  

void ledRedGreen(char ledStatus);