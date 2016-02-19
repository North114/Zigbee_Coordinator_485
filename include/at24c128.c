#include "at24c128.h"

/* 外部定义的函数,省去了include那个头文件 */
//extern void USART0_Send_Byte(unsigned char);

/*
Write 1 byte data to EEPROM
*/
unsigned char WriteEEPROM(unsigned char DevAddr,unsigned int MemAddr,unsigned char data)
{
	/* Start TWI */
	Start();
	Wait();
	if(TestACK()!=START)
	{
	   return 0;
	}
	/* Write Device Address */
	Writebyte((DevAddr<<1)|(Write));
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
	   return 0;
	}
	/* Write Memory Address High Byte) */
	Writebyte(MemAddr>>8);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Memory Address Low Byte) */
	Writebyte(MemAddr&0xff);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Data to EEPROM */
	Writebyte(data);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
	   return 0;
	}
	Stop();
	_delay_ms(15);
	
	return 1;
}
/*
Write n byte data to EEPROM
*/
unsigned char Write_EEPROM_Block(unsigned char DevAddr,unsigned int MemAddr,unsigned char *p,unsigned char num)
{
	unsigned char i;	
	cli();		//Disable Globle Interrupt
	/* Start TWI */
	Start();
	Wait();
	if(TestACK()!=START)
	{
	   return 0;
	}
	/* Write Device Address */
	Writebyte((DevAddr<<1)|(Write));
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
	   return 0;
	}
	/* Write Memory Address High Byte) */
	Writebyte(MemAddr>>8);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Memory Address Low Byte) */
	Writebyte(MemAddr&0xff);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Data to EEPROM */
	for(i = 0;i < num;i++){
		Writebyte(*(p+i));
		Wait();
		if(TestACK()!=MT_DATA_ACK)
		{
	   		return 0;
		}
	}

	Stop();
	_delay_ms(5);
	sei();            //Enable Global Interrupt
	return 1;
}
/*
Read 1 byte data from EEPROM
*/
unsigned char ReadEEPROM(unsigned char DevAddr,unsigned int MemAddr)
{
	unsigned char data;
	/* Start TWI */
	Start();
	Wait();
	if(TestACK()!=START)
	{
		return 0;
	}	
	/* Write Device Address */
	Writebyte((DevAddr<<1)|(Write));//SLA+W
	Wait();
	if(TestACK()!=MT_SLA_ACK)
	{
		return 0;
	}
	/* Write Memory Address High Byte) */
	Writebyte(MemAddr>>8);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Memory Address Low Byte) */
	Writebyte(MemAddr&0xff);
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
	/* Write Device Address(read format) */
	Writebyte((DevAddr<<1)|(Read));//SLW+R
	Wait();
	if(TestACK()!=MR_SLA_ACK)//
	{
		return 0;
	}

	/* added for certian purpose */
	TWCR=(1<<TWINT)|(1<<TWEN);
	Wait();

	/* Receive Data(1 byte only) */

	data = TWDR;

	/* Stop TWI */

	Stop();

	_delay_ms(5);
	return data;
}
/*
Read n byte data from EEPROM
*/
unsigned char Read_EEPROM_Block(unsigned char DevAddr,unsigned int MemAddr,unsigned char *p,unsigned char num)
{
	//unsigned char data;
	unsigned char i = 0;
	cli();		//Disable Global Interrupt
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
	/* Write Memory Address High Byte) */
	Writebyte(MemAddr>>8);
	Wait();
	if(TestACK()!=MT_DATA_ACK)
	{
		return 0;
	}
	/* Write Memory Address Low Byte) */
	Writebyte(MemAddr&0xff);
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
	/* Write Device Address(read format) */
	Writebyte(DevAddr + 1);//SLW+R
	Wait();
	if(TestACK()!=MR_SLA_ACK)//
	{
	   return 0;
	}
	/* Read MultiByte From EEPROM */
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
	
	sei();		//Enable Global Interrupt
	return 1;

}
/*
** 初始化EEPROM
*/
void InitEEPROM() {
	WriteEEPROM(AT24C128,0,1);//Coordinator ID - 1
	WriteEEPROM(AT24C128,15,0);//high byte of FirstReadByteAddr
	WriteEEPROM(AT24C128,16,34);//low byte of FirstReadByteAddr
	WriteEEPROM(AT24C128,17,0);//high byte of LastReadByteAddr
	WriteEEPROM(AT24C128,18,33);//low byte of LastReadByteAddr
	WriteEEPROM(AT24C128,19,0);//high byte of  FirstUnReadByteAddr
	WriteEEPROM(AT24C128,20,34);//low byte of FirstUnReadByteAddr
	WriteEEPROM(AT24C128,21,0);//high byte of LastUnReadByteAddr
	WriteEEPROM(AT24C128,22,33);//low byte of LastUnReadByteAddr
	WriteEEPROM(AT24C128,23,0);//EEPROM Full Byte
}
/*
** 读取EEPROM值并发送
*/
void ReadInitEEPROMAddr() {
	unsigned int temp;

	USART0_Send_Byte(0x30 + ReadEEPROM(AT24C128,0));
	USART0_Send_Byte(0x0A);
	
	temp = 256*ReadEEPROM(AT24C128,15) + ReadEEPROM(AT24C128,16);

	USART0_Send_Byte(0x30 + temp/10000);
	USART0_Send_Byte(0x30 + (temp%10000)/1000);
	USART0_Send_Byte(0x30 + (temp%1000)/100);
	USART0_Send_Byte(0x30 + (temp%100)/10);
	USART0_Send_Byte(0x30 + (temp%10));
	USART0_Send_Byte(0x0A);//line feed
	_delay_ms(5);

	temp = 256*ReadEEPROM(AT24C128,17) + ReadEEPROM(AT24C128,18);

	USART0_Send_Byte(0x30 + temp/10000);
	USART0_Send_Byte(0x30 + (temp%10000)/1000);
	USART0_Send_Byte(0x30 + (temp%1000)/100);
	USART0_Send_Byte(0x30 + (temp%100)/10);
	USART0_Send_Byte(0x30 + (temp%10));
	USART0_Send_Byte(0x0A);//line feed
	_delay_ms(5);

	temp = 256*ReadEEPROM(AT24C128,19) + ReadEEPROM(AT24C128,20);

	USART0_Send_Byte(0x30 + temp/10000);
	USART0_Send_Byte(0x30 + (temp%10000)/1000);
	USART0_Send_Byte(0x30 + (temp%1000)/100);
	USART0_Send_Byte(0x30 + (temp%100)/10);
	USART0_Send_Byte(0x30 + (temp%10));
	USART0_Send_Byte(0x0A);//line feed
	_delay_ms(5);

	temp = 256*ReadEEPROM(AT24C128,21) + ReadEEPROM(AT24C128,22);

	USART0_Send_Byte(0x30 + temp/10000);
	USART0_Send_Byte(0x30 + (temp%10000)/1000);
	USART0_Send_Byte(0x30 + (temp%1000)/100);
	USART0_Send_Byte(0x30 + (temp%100)/10);
	USART0_Send_Byte(0x30 + (temp%10));
	USART0_Send_Byte(0x0A);//line feed
	_delay_ms(5);

	USART0_Send_Byte(0x30 + ReadEEPROM(AT24C128,23));
	USART0_Send_Byte(0x0A);//line feed
}
