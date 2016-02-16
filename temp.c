#ifndef F_CPU
	#define F_CPU 16000000UL  /* 16 MHz CPU clock */
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ds1307.h"
#include "at24c128.h"
#include "usart.h"
#include "init.h"

/* DS1307 Relatted Variable Defination */
unsigned char CurrentTime[7];

int main(){
	TWI_Init();
	USART0_Init(38400);
	USART1_Init(38400);
	Timer0_Init();
    InitWatchDogTimer();

	unsigned int t = ReadDS1307(DS1307,0);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,1);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,2);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,3);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,4);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,5);
	USART0_Send_Byte(t);
	t = ReadDS1307(DS1307,6);
	USART0_Send_Byte(t);

	t = ReadEEPROM(AT24C128,0);
	USART0_Send_Byte(t);

	while(1) {

	}

	return 0;
}