#ifndef F_CPU
	#define F_CPU 16000000UL  /* 16 MHz CPU clock */
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "include/ds1307.h"
#include "include/at24c128.h"
#include "include/usart.h"
#include "include/init.h"

/* DS1307 Relatted Variable Defination */
unsigned char CurrentTime[7];
/*
** Main Function
*/
int main(){
    volatile unsigned int t;
    volatile unsigned int r,second;

	USART0_Init(38400);
	USART1_Init(38400);
    //Timer0_Init();
    //InitWatchDogTimer();
    
	TWI_Init();

    _delay_ms(500);
    USART0_Send_Byte(0x50);
    
    InitDate(0x14,0x02,0x13,0x05);

	//USART0_Send_Byte(t);

	while(1) {
        second = ReadDS1307(DS1307,0x00);
		r = Read_Current_Time(DS1307,CurrentTime,7);
		
		if(r != 0){
			/* Read out current time every 10 seconds,then transmit data through bluetooth */
			_delay_ms(5000);//delay 10 seconds
			//USART0_Send_Byte(0x30 + second/16);
			//USART0_Send_Byte(0x30 + second%16);
			//USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[0]/16);
			USART0_Send_Byte(0x30 + CurrentTime[0]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[1]/16);
			USART0_Send_Byte(0x30 + CurrentTime[1]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[2]/16);
			USART0_Send_Byte(0x30 + CurrentTime[2]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[3]/16);
			USART0_Send_Byte(0x30 + CurrentTime[3]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[4]/16);
			USART0_Send_Byte(0x30 + CurrentTime[4]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[5]/16);
			USART0_Send_Byte(0x30 + CurrentTime[5]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[6]/16);
			USART0_Send_Byte(0x30 + CurrentTime[6]%16);
			USART0_Send_Byte(0x0A);//line feed
	    }   
    }
	return 0;
}
