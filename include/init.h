#ifndef INT_H
	#define INT_H

	#include <avr/io.h>

	#define T0IniVal 55
	
	void InitWatchDogTimer();
	void Timer0_Init();
#endif