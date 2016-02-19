#include "usart.h"

/*
Initialize USART0
*/
void USART0_Init(unsigned int baud) {
	UCSR0A = 0x00;//defalut value
	UCSR0B = 0x00;//USART Control and Status Register B 		    //¿ØÖÆ¼Ä´æÆ÷ÇåÁã
	UCSR0C = 3 << UCSZ00;//8 bit data
    /* Add Odd Parity(Page 193) */
    //UCSR0C |= 2 << UPM00;
                                                        //Ñ¡ÔñUCSRC£¬Òì²½Ä£Ê½£¬½ûÖ¹                        
                                                     //   Ð£Ñé£¬1Î»Í£Ö¹Î»£¬8Î»Êý¾ÝÎ»
	baud = F_CPU/16/baud - 1	;   //²¨ÌØÂÊ×î´óÎª65K
	UBRR0L = baud; 					     	  
	UBRR0H = baud>>8; 		   //ÉèÖÃ²¨ÌØÂÊ
   
	UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0); //½ÓÊÕ¡¢·¢ËÍÊ¹ÄÜ£¬½ÓÊÕÖÐ¶ÏÊ¹ÄÜ
   
	//SREG = BIT(7);	       //È«¾ÖÖÐ¶Ï¿ª·Å
	DDRD |= 0x02;	           //ÅäÖÃTX0 pin(PD1) ÎªÊä³ö£¨ºÜÖØÒª£©
}
/*
Initialize USART1
*/
void USART1_Init(unsigned int baud)
{
	UCSR1A = 0x00;
	UCSR1B = 0x00;//USART Control and Status Register B 		    //¿ØÖÆ¼Ä´æÆ÷ÇåÁã
	UCSR1C = 3<<UCSZ10;//8 bit data
                                                        //Ñ¡ÔñUCSRC£¬Òì²½Ä£Ê½£¬½ûÖ¹                        
                                                     //   Ð£Ñé£¬1Î»Í£Ö¹Î»£¬8Î»Êý¾ÝÎ»
	baud = F_CPU/16/baud - 1	;   //²¨ÌØÂÊ×î´óÎª65K
	UBRR1L = baud; 					     	  
	UBRR1H = baud>>8; 		   //ÉèÖÃ²¨ÌØÂÊ
	
	UCSR1B = (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1); //½ÓÊÕ¡¢·¢ËÍÊ¹ÄÜ£¬½ÓÊÕÖÐ¶ÏÊ¹ÄÜ
   	
	//SREG = BIT(7);	       //È«¾ÖÖÐ¶Ï¿ª·Å
	DDRD |= 0x08;	           //ÅäÖÃTX1 pin(PD3) ÎªÊä³ö£¨ºÜÖØÒª£©
}

/*
** Send Data Through USART0(Bluetooth Module)
*/
inline void USART0_Send_Byte(unsigned char data)
{
	/* waitting for a empty USART Data Register */
	while(!(UCSR0A&(1<<UDRE0))) ;
	UDR0 = data;//USART Data Register
   
	/* waitting for USART Transmit Complete */
	while(!(UCSR0A&(1<<TXC0)));
	UCSR0A |= 1<<TXC0;//set TXC bit manually
}
/*
** Send Data Through USART1(Zigbee Module)
*/
void USART1_Send_Byte(unsigned char data)
{
	/* waitting for a empty USART Data Register */
	while(!(UCSR1A&(1<<UDRE1))) ;
	UDR1 = data;//USART Data Register
   
	/* waitting for USART Transmit Complete */
	while(!(UCSR1A&(1<<TXC1)));
	UCSR1A |= 1<<TXC1;//set TXC bit manually
}
