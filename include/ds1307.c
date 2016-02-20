#include "ds1307.h"

/* 在其他头文件中定义的函数 */
//extern unsigned char WriteEEPROM(unsigned char DevAddr,unsigned int MemAddr,unsigned char data);

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
unsigned char WriteDS1307(unsigned char DevAddr,unsigned int MemAddr,unsigned char data)
{   
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
** 初始化日期时间
*/
void InitDateTime(unsigned char s,unsigned char m,unsigned char h,unsigned char w,unsigned char d,unsigned char mon,unsigned char y) {
	WriteDS1307(DS1307,0x00,s);//second
	_delay_ms(10);
	WriteDS1307(DS1307,0x01,m);//minute
	_delay_ms(10);
	WriteDS1307(DS1307,0x02,h);//hour
	_delay_ms(10);
	WriteDS1307(DS1307,0x03,w);//day in a week
	_delay_ms(10);
	WriteDS1307(DS1307,0x04,d);//day
	_delay_ms(10);
	WriteDS1307(DS1307,0x05,mon);//month
	_delay_ms(10);
	WriteDS1307(DS1307,0x06,y);//year
	_delay_ms(10);
}
/*
** 初始化日期
*/
void InitDate(unsigned char y,unsigned char mon,unsigned char d,unsigned char w) {
	WriteDS1307(DS1307,0x03,w);//day in a week
	_delay_ms(10);
	WriteDS1307(DS1307,0x04,d);//day
	_delay_ms(10);
	WriteDS1307(DS1307,0x05,mon);//month
	_delay_ms(10);
	WriteDS1307(DS1307,0x06,y);//year
	_delay_ms(10);
}
/*
** 初始化时间
*/
void InitTime(unsigned char h,unsigned char m,unsigned char s){
	WriteDS1307(DS1307,0x00,s);//second
	_delay_ms(10);
	WriteDS1307(DS1307,0x01,m);//minute
	_delay_ms(10);
	WriteDS1307(DS1307,0x02,h);//hour
	_delay_ms(10);
}
