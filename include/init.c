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
/*
Clear Bit(Set bit to 0)
*/
inline unsigned char setBit(unsigned char d,unsigned char n) {
	return (d | (1<<n));
}
/*
Set Bit(set bit to 1)
*/
/* Example : data = clearBit(data,1),means clear 1st bit of data(note data holds 1 byte) */
inline unsigned char clearBit(unsigned char d,unsigned char n) {
	return (d & (~(1<<n)));
}
/*
** Turn On LED
*/
void LEDON() {
    PORTC |= 0x80; //Turns ON LEDs
}
/*
** Turn Off LED
*/
void LEDOFF() {
    PORTC &= ~(0x80); //Turns OFF LEDs
}
/*
Check out Button Status
*/
unsigned char CheckButtonStatus(unsigned char oldStatus) {
    unsigned char temp ;
    unsigned char result;

	temp = PINC & 0x30;//read button status

    if(temp & (1 << 5)) {
        /* Switch to GPRS*/
        PORTD = setBit(PORTD,5); 
        PORTD = clearBit(PORTD,4);
        result = 1;
        if(result != oldStatus) {
            /* reset baud rate and init USART0 */
            USART0_Reset_BaudRate(38400);
        }
    } else {
        /* Switch to RS485 bus */
        PORTD = clearBit(PORTD,5);
        PORTD = setBit(PORTD,4);
        //PORTD &= 0xDF; //clear bit PD5 or clearBit(PORTD,5)
        //PORTD |= 0x10; //set bit PD4 or setBit(PORTD,4)
        result = 0;
        if(result != oldStatus) {
            /* reset baud rate and init USART0 */
            USART0_Reset_BaudRate(2400);
        }
    }

    return result;
}
