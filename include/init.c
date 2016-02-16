#include "init.h"
/*
Initialize Watch Timer Dog
*/
void InitWatchDogTimer() {
	/* Start Timed Sequence */
	WDTCSR |= (1<<WDCE)|(1<<WDE);//set WDCE to change WDE and prescaler
	/* Set New Prescaler Time-Out Value */
	WDTCSR = (1<<WDE)|(1<<WDP3)|(1<<WDP0);	//Time-Out is 8 seconds,system reset mode(@page 61)
	//Watch Dog Timer in High Fuse Bit(WDTON = 1)
}
/*
Initialize Timer0
*/
void Timer0_Init() {
	TCCR0B = (1<<CS02);//PRE-SCALE : 256 (@page 110)
	TIMSK0 = (1<<TOIE0);//ENABLE TIMER 0 OVERFLOW INTERRUPT(@page 111)
	
	TCNT0 = T0IniVal;//Timing/Counter Register
}