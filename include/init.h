#ifndef INT_H
	#define INT_H

	#include <avr/io.h>
    #include "usart.h"    

	#define T0IniVal 55
    
    #define FEED_DOG (__asm__ __volatile__ ("wdr"))

	void InitWatchDogTimer();
	void Timer0_Init();
    inline unsigned char setBit(unsigned char d,unsigned char n);
    inline unsigned char clearBit(unsigned char d,unsigned char n);
    void LEDON();
    void LEDOFF();
    unsigned char CheckButtonStatus(unsigned char);
#endif
