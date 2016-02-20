#ifndef F_CPU
	#define F_CPU 16000000UL  /* 16 MHz CPU clock */
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#include "include/ds1307.h"
//#include "include/iic.h"
#include "include/at24c128.h"
#include "include/usart.h"
#include "include/init.h"

#define DS1307 0x68
/* DS1307 Relatted Variable Defination */
unsigned char CurrentTime[7];
/*
** Read 1 data byte from DS1307
*/
unsigned char ReadDS1307(unsigned char DevAddr,unsigned char RegAddr)
{
	unsigned char data;
	
	DevAddr = (DevAddr<<1)|(Write);//????????????????
	/* Start TWI */
	Start();
	Wait();
	if(TestACK()!=START)
	{
	   return 0;
	}
	/* Write Device Address */
	Writebyte(DevAddr);//SLA+W
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
	   return 0;
	}
	/* Write Register Address 0x00) */
	Writebyte(RegAddr);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
	   return 0;
	}

	/* Restart */	

	Start();
	Wait();
	if(TestACK()!=ReStart)
	{
	   return 0;
	}
	Writebyte(DevAddr + 1);//SLW+R
	Wait();
	if(TestACK()!=MR_SLA_ACK)//
	{
	   return 0;
	}
	
	/* added for certian purpose */
	//TWCR=(1<<TWINT)|(1<<TWEN);
	ResetACK();
	Wait();
	if(TestACK()!=MR_DATA_NACK)//
	{
	   return 0;
	}
	/* Receive Data(1 byte only) */
	data = TWDR;
	
	Stop();

	_delay_ms(5);
	
	return data;

}
/*
** Write 1 byte data to DS1307
*/
unsigned char WriteDS1307(unsigned char DevAddr,unsigned int MemAddr,unsigned char data) {

	Start();
	Wait();
	if(TestACK()!=START)
	{
	   return 0;
	}
	Writebyte(DevAddr << 1 | Write);
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
	   return 0;
	}
	Writebyte(MemAddr);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
	   return 0;
	}
	Writebyte(data);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
	   return 0;
	}
	Stop();
	_delay_ms(10);
	
	return 1;
}
/*
** Read All Time data
*/
unsigned char Read_Current_Time(unsigned char DevAddr,unsigned char *p,unsigned char num)
{
	//unsigned char data;
	unsigned char i = 0;
	cli();		//Diaable Global Interrupt
	DevAddr = (DevAddr<<1)|(Write);//????????????????
	/* Start TWI */
	Start();
	Wait();
	if(TestACK()!=START)
	{
	   return 0;
	}
	/* Write Device Address */
	Writebyte(DevAddr);//SLA+W
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
	   return 0;
	}
	/* Write Register Address 0x00) */
	Writebyte(0);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
	   return 0;
	}

	/* Restart */	

	Start();
	Wait();
	if(TestACK()!=ReStart)
	{
	   return 0;
	}
	Writebyte(DevAddr + 1);//SLW+R
	Wait();
	if(TestACK()!=MR_SLA_ACK)//
	{
	   return 0;
	}

	/* Rstart TWI */
	//TWCR=(1<<TWINT)|(1<<TWEN);
	//Wait();

	/* Read MultiByte From DS1307 ???? */
	for(i = 0;i < (num-1);i++){
		/* Receive Data(1 byte only) */
		SetACK();
		Wait();
		if(TestACK()!=MR_DATA_ACK)//
		{
	   		return 0;
		}
		*(p+i) = TWDR;
		//to do (ACK)????	
	}
	ResetACK();
	Wait();
	if(TestACK()!=MR_DATA_NACK)//
	{
	   	return 0;
	}
	*(p+num-1) = TWDR;

	Stop();

	_delay_ms(5);
	
	sei();//Enable Global Interrupt
	return 1;

}


/*
** Main Function
*/
int main(){
    volatile unsigned int t;
    volatile unsigned int r,second;

	USART0_Init(38400);
	USART1_Init(38400);
// Timer0_Init();
//    InitWatchDogTimer();
    
	TWI_Init();

    _delay_ms(500);
    USART0_Send_Byte(0x50);
    
    WriteDS1307(DS1307,0x02,0x06);
    _delay_ms(10);
    WriteDS1307(DS1307,0x01,0x05);
    _delay_ms(10);
    WriteDS1307(DS1307,0x02,0x01);
    _delay_ms(10);
    WriteDS1307(DS1307,0x03,0x01);
    _delay_ms(10);
    WriteDS1307(DS1307,0x04,0x22);
    _delay_ms(10);
    WriteDS1307(DS1307,0x05,0x12);
    _delay_ms(10);
    WriteDS1307(DS1307,0x06,0x16);
    _delay_ms(10);
    
    //InitDateTime(0x16,0x02,0x13,0x05,0x10,0x03,0x40);

	//USART0_Send_Byte(t);

	while(1) {
        second = ReadDS1307(DS1307,0x00);
		r = Read_Current_Time(DS1307,CurrentTime,7);
		
		if(r != 0){
			/* Read out current time every 10 seconds,then transmit data through bluetooth */
			_delay_ms(5000);//delay 10 seconds
			//USART0_Send_Byte(0x30 + second/16);
			//USART0_Send_Byte(0x30 + second%16);
			//USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[0]/16);
			USART0_Send_Byte(0x30 + CurrentTime[0]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[1]/16);
			USART0_Send_Byte(0x30 + CurrentTime[1]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[2]/16);
			USART0_Send_Byte(0x30 + CurrentTime[2]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[3]/16);
			USART0_Send_Byte(0x30 + CurrentTime[3]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[4]/16);
			USART0_Send_Byte(0x30 + CurrentTime[4]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[5]/16);
			USART0_Send_Byte(0x30 + CurrentTime[5]%16);
			USART0_Send_Byte(0x0A);//line feed
			USART0_Send_Byte(0x30 + CurrentTime[6]/16);
			USART0_Send_Byte(0x30 + CurrentTime[6]%16);
			USART0_Send_Byte(0x0A);//line feed
	    }   
    }
	return 0;
}
