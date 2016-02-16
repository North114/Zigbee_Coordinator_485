/*
CPU : ATmega644pA
Frequency : 16MHz

Function : Transmit Received Data from USART

*/
#define F_CPU 16000000UL  /* 16 MHz CPU clock */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define READ485 (PORTB &= 0xEF)
#define WRITE485 (PORTB |= 0x10)

volatile unsigned char rdata;
volatile unsigned char flag = 0;
//bool flag;
/*
Initialize USART0
*/
void USART0_Init(unsigned int baud)
{
	UCSR0A = 0x00;//defalut value
	UCSR0B = 0x00;//USART Control and Status Register B 		    //控制寄存器清零
	UCSR0C = 3<<UCSZ00;//8 bit data
                                                        //选择UCSRC，异步模式，禁止                        
                                                     //   校验，1位停止位，8位数据位
	baud = F_CPU/16/baud - 1;   //波特率最大为65K
	UBRR0L = baud; 					     	  
	UBRR0H = baud>>8; 		   //设置波特率
   
	UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0); //接收、发送使能，接收中断使能
   
	//SREG = BIT(7);	       //全局中断开放
	DDRD |= 0x02;	           //配置TX0 pin(PD1) 为输出（很重要）
}
/*
Initialize USART1
*/
void USART1_Init(unsigned int baud)
{
	UCSR1A = 0x00;
	UCSR1B = 0x00;//USART Control and Status Register B 		    //控制寄存器清零
	UCSR1C = 3<<UCSZ10;//8 bit data
                                                        //选择UCSRC，异步模式，禁止                        
                                                     //   校验，1位停止位，8位数据位
	baud = F_CPU/16/baud - 1;   //波特率最大为65K
	UBRR1L = baud; 					     	  
	UBRR1H = baud>>8; 		   //设置波特率
	
	UCSR1B = (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1); //接收、发送使能，接收中断使能
   	
	//SREG = BIT(7);	       //全局中断开放
	DDRD |= 0x08;	           //配置TX1 pin(PD3) 为输出（很重要）
}
/*
Send Data Through USART0
*/
void UART0_Send_Byte(unsigned char data)
{
	/* waitting for a empty USART Data Register */
	while(!(UCSR0A&(1<<UDRE0))) ;
	UDR0 = data;//USART Data Register
   
	/* waitting for USART Transmit Complete */
	while(!(UCSR0A&(1<<TXC0)));
	UCSR0A |= 1<<TXC0;//set TXC bit manually
}
/*
Send Data Through USART1
*/
void UART1_Send_Byte(unsigned char data)
{
	/* waitting for a empty USART Data Register */
	while(!(UCSR1A&(1<<UDRE1))) ;
	UDR1 = data;//USART Data Register
   
	/* waitting for USART Transmit Complete */
	while(!(UCSR1A&(1<<TXC1)));
	UCSR1A |= 1<<TXC1;//set TXC bit manually
}
/*
USART0 Receive Interrupt Service Routing
*/
ISR(USART0_RX_vect)//USART Receive Complete Vector
{
	unsigned char temp;	
	UCSR0B &= (~(1<<RXCIE0));//disable receiver interrupt(reset bit)
	temp = UDR0;//read data
	
	flag = 1;//set receive flag
	
	WRITE485;
	UART0_Send_Byte(temp);
	READ485;

	UCSR0B |= (1<<RXCIE0);//re-enable receiver interrupt(set bit)
}
/*
USART1 Receive Interrupt Service Routing
*/
ISR(USART1_RX_vect)//USART Receive Complete Vector
{
	unsigned char temp;	
	UCSR1B &= (~(1<<RXCIE1));//disable receiver interrupt(reset bit)
	temp = UDR1;//read data
	//flag = 1;//set receive flag
	
	/* waitting for a empty USART Data Register */
	while(!(UCSR1A&(1<<UDRE1))) ;
	UDR1 = temp;//USART Data Register
   
	/* waitting for USART Transmit Complete */
	while(!(UCSR1A&(1<<TXC1)));
	UCSR1A |= 1<<TXC1;//set TXC bit manually

	UCSR1B |= (1<<RXCIE1);//re-enable receiver interrupt(set bit)
}

void initIOfor485Bus() {
	/* 0011 0000 */
    DDRD |= 0x30; //make portd(4:5) output;
	/* 1101 1111 
	   we need to set and clear bit one ny one !!!!!!!
	*/
    PORTD &= 0xDF; //clear bit PD5
    PORTD |= 0x10; //set bit PD4

	/* make 485 enable pin Output pin */
	DDRB |= 0x10;
}

int main()
{
	/*
	  --  -----  ---
	  |	B      A   |
	  | left  right|
	  |____________|

	*/
	/* Multiplex Port */
    USART0_Init(2400);//Initialize USART0 with baud rate of 9600
    /* Zigbee Port */
	USART1_Init(38400);//Initialize USART1 with baud rate of 9600
    
	initIOfor485Bus();

    _delay_ms(10);
    sei();                     //Enable Gloabal Interrupt
	
	WRITE485;
	//_delay_ms(100);
    UART0_Send_Byte(0x36);//for debug use
    UART0_Send_Byte(0x36);//for debug use
	READ485;

    while(1) {
		__asm__ __volatile__("nop");
		if(1 == flag) {
			WRITE485;
			//_delay_ms(100);
			UART0_Send_Byte(0x55);//for debug use
			READ485;
			flag = 0;		
		}
    }

    return 0;

}
