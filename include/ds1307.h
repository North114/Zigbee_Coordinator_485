#ifndef DS1307_H
	#ifndef F_CPU
		#define F_CPU 16000000UL  /* 16 MHz CPU clock */
	#endif

	#include "iic.h"
	#include <util/delay.h>
	#include <avr/interrupt.h>
	

	#define DS1307_H
	#define DS1307 0x68//define 7-bit slave device address.
	
	/* Function Declaration */
	unsigned char ReadDS1307(unsigned char DevAddr,unsigned char RegAddr);
	unsigned char Read_Current_Time(unsigned char DevAddr,unsigned char *p,unsigned char num);
	void InitDateTime(unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char,unsigned char);
    void InitDate(unsigned char,unsigned char,unsigned char,unsigned char);
    void InitTime(unsigned char,unsigned char,unsigned char);
#endif
