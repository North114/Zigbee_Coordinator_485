#ifndef IIC_H
	#include <avr/io.h>

	#define IIC_H
	/* TWI Related Macro */
	/* status codes for master transmiter mode */
	#define  START  0X08//A TWI_Start condition has been transmitted
	#define  ReStart 0x10//A repeated Start condition has been transmitted
	#define  MT_SLA_ACK  0X18//(Master Transmit SLave Address ACK)slave+w has been transmitted;ACK has been received
	#define  MR_SLA_ACK 0x40//Master Receive Slave Address ACK
	#define  MT_DATA_ACK  0X28//(Master Transmit DATA ACK)data byte has been transmitted;ACK has been received
	#define  MR_DATA_ACK 0x50//Master Receive DATA ACK
	#define  MR_DATA_NACK 0X58//Master Receive DATA Not ACK

	#define Read 1
	#define Write 0

	#define Start() (TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN))	//²úÉúSTARTÐÅºÅ
	#define Stop() (TWCR=(1<<TWINT)|(1<<TWSTO)|(1<<TWEN))	//²úÉúSTOPÐÅºÅ
	#define Wait() while(!(TWCR&(1<<TWINT)))				//µÈ´ýµ±Ç°²Ù×÷Íê³É
	#define TestACK() (TWSR&0xF8)							//È¡³ö×´Ì¬Âë(TWI Status Register)
	#define SetACK() (TWCR|=(1<<TWEA))						//²úÉúACK
	#define ResetACK() (TWCR=(1<<TWINT)|(1<<TWEN))			//²úÉúNACK
	#define Writebyte(twi_d) {TWDR=(twi_d);TWCR=(1<<TWINT)|(1<<TWEN);}	//·¢ËÍÒ»¸ö×Ö½Ú£¨twi_dÎªÐ´ÈëµÄÊý¾Ý£©
	
	/*
		Initialize TWI
	*/
	#define TWI_Init() {\
	    TWBR = 0x20;\
		TWCR = 0x44;\
		TWSR = 0;\
	}

#endif
