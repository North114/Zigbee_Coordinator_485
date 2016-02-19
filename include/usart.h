#ifndef USART_H
	#define USART_H
    
    #ifndef F_CPU
        #define F_CPU 16000000UL
    #endif
    #include <avr/io.h>
	void USART0_Init(unsigned int baud);
	void USART1_Init(unsigned int baud);

	inline void USART0_Send_Byte(unsigned char data);
	void USART1_Send_Byte(unsigned char data);
#endif
