#ifndef DS1307_H
	#ifndef F_CPU
		#define F_CPU 16000000UL  /* 16 MHz CPU clock */
	#endif
    #include <util/delay.h>
    #include <avr/interrupt.h>
    #include "iic.h"
	
    #define DS1307_H
	#define DS1307 0x68//define 7-bit slave device address.
	#define SECOND 0x00
    #define MINUTE 0x01
    #define HOUR 0x02
    #define WEEKDAY 0x03
    #define DATE 0x04
    #define MONTH 0x05
    #define YEAR 0x06

	/* Function Declaration */
	unsigned char ReadDS1307(unsigned char DevAddr,unsigned char RegAddr);
    unsigned char WriteDS1307(unsigned char DevAddr,unsigned int MemAddr,unsigned char data);
	unsigned char Read_Current_Time(unsigned char DevAddr,unsigned char *p,unsigned char num);
	void InitDateTime(unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char);
    void InitDate(unsigned char,unsigned char,unsigned char,unsigned char);
    void InitTime(unsigned char,unsigned char,unsigned char);
#endif
