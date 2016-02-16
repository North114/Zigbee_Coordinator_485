#ifndef H_485
	#define H_485
	
	#define READ485 (PORTB &= 0xEF)
	#define WRITE485 (PORTB |= 0x10)

#endif
