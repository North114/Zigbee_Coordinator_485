#ifndef AT24C128_H
	#define AT24C128_H

	#ifndef F_CPU
		#define F_CPU 16000000UL  /* 16 MHz CPU clock */
	#endif
	
	#include <util/delay.h>
	#include <avr/interrupt.h>
    #include "iic.h"
    #include "usart.h"
	
    #define AT24C128 0x50//define 7-bit slave device address.
	#define BlockLength 15
	#define EEpromSize 16384//Address Range: 0 ~ 16383
	#define ReservedByteNum 34

	unsigned char WriteEEPROM(unsigned char DevAddr,unsigned int MemAddr,unsigned char data);
	unsigned char Write_EEPROM_Block(unsigned char DevAddr,unsigned int MemAddr,unsigned char *p,unsigned char num);

	unsigned char ReadEEPROM(unsigned char DevAddr,unsigned int MemAddr);
	unsigned char Read_EEPROM_Block(unsigned char DevAddr,unsigned int MemAddr,unsigned char *p,unsigned char num);

	void InitEEPROM();
	void ReadInitEEPROMAddr();

#endif
