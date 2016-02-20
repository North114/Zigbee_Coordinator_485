/*
CPU : ATmega644pA
Frequency : 16MHz

Function : Zigbee Coordinator(Version 01)

Added Function:Read Real-Time Data of all Router or Specified Router

Some Volatile need to be added before variable declare

changing log:

description:changing recent data meaning,now,recent data means 'today's' data;
date:01-26-2015

description:close global interrupt when we use twi;
date:01-27-2015

description:close global interrupt when we store received data into eeprom
date:03-17-2015
*/
#ifndef F_CPU
	#define F_CPU 16000000UL  /* 16 MHz CPU clock */
#endif
/* System Defined Head files */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* Self Defined Head files */
#include "include/ds1307.h"
#include "include/at24c128.h"
#include "include/usart.h"
#include "include/init.h"
#include "include/485.h"

//#define DEBUG
//#define TestUSART1
/* Zigbee Related Macro */
/** StartByte_Zigbee + UserIDByte + LeakageValueByteMSB + LeakageValueByteLSB
+ VoltageMSB + VoltagelSB + EndByte_Zigbee**/
#define recBufferSize_Zigbee 25// larger than PackLen
#define Zigbee_PackLen 13 // Length + Address(6) + DeviceId + Current(2) + Voltage(2) + EventType
#define Zigbee_AckLen 12

#define StartByte_Zigbee 0xAA
#define EndByte_Zigbee 0x75 //End byte should less than 128,since it's a character
#define ZigbeeQueryByte 0xCC //query command byte
//#define RouterNum 80       //Total device number in a PAN

/* 485 Related Macro */
#define recBufferSize_485 256// larger than PackLen
#define replyBufferSize_485 256
#define StartByte_485 0x68
#define EndByte_485 0x16 //end byte should less than 128,since it's a character
#define DecodeByte 0x33
#define HISTORY_CACHE_SIZE 10
#define MAX_UCHAR 255
#define MAX_UINT 65535
#define ONCHIP_EEPROM_SIZE (2*1024) //?ñ?????ȥ??????
#define MONITOR_EEPROM_DOWN 100  //??ʼ??ַ
#define MONITOR_EEPROM_SIZE (4*24*4*2) //?ֽ???
#define ADDR_LENGTH 6 //router address length
#define MaxRouterNumber 50 //as name
#define RouterDataUnitSize 200 //each router has 200 byte space
#define EEPROM_OFFSET 100 //byte reserved
#define RouterDataItemLength 10 //length of every item of router data
#define MYINT_MAX 65535

#define CACHE_TIME 250
#define CACHE_SPACE 200
/* Function Declaration */
/* DS1307 Related Variable Declaration */
unsigned char CurrentTime[7];

/* AT24C128 Relatted Variable Defination */
unsigned char W_EEprom_Array[BlockLength],R_EEprom_Array[BlockLength];
unsigned int FirstReadByteAddr = 0, LastReadByteAddr = 0;
unsigned int FirstUnReadByteAddr = 0,LastUnReadByteAddr = 0;
unsigned char EEpromFull;

/* Zigbee Relatted Variable Defination */
volatile unsigned char startFlag_Zigbee = 0;
volatile unsigned char recFlag_Zigbee = 0;//add a 'volatile' is very impportant
//we can also choose _Bool 
volatile unsigned char index_Zigbee = 0;
volatile unsigned char recNum_Zigbee = 0;
volatile unsigned char recBuffer_Zigbee[recBufferSize_Zigbee];

volatile unsigned char ACK_Zigbee[Zigbee_AckLen] = {StartByte_Zigbee,0x0A,0,0,0,0,0,0,0x04,0,0,EndByte_Zigbee};


/* 485 Relatted Variable Defination
   format : startByte + address + startByte + controlByte + lengthByte + DATA + checkSum + endByte
*/
volatile unsigned char startFlag_485 = 0;
volatile unsigned char recFlag_485 = 0;//add a 'volatile' is very impportant
volatile unsigned char index_485 = 0,recNum_485 = 0;
volatile unsigned char myaddr[] = {0x12,0x34,0x56,0x78,0x90,0x12};//LSB -- MSB
volatile unsigned char password[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
volatile unsigned char recBuffer_485[recBufferSize_485];
volatile unsigned char recData_485[recBufferSize_485];
volatile unsigned char replyBuffer_485[replyBufferSize_485];
volatile unsigned char ZigbeeTransmitBuf[10];

/* ֡?ֽ? */
volatile unsigned char CheckSum_485 = 0;
volatile unsigned char ControlByte_485 = 0;
volatile unsigned char DataLength_485 = 0;

volatile unsigned char RealTimeQuery = 0;

/* ??ѯ???????ر??? */
volatile unsigned char ThisHour;//??ǰСʱ??
volatile unsigned char oneMinuteCount = 0;
volatile unsigned char oneMinuteFlag = 0;
volatile unsigned char monitorIndex = MONITOR_EEPROM_DOWN;
/* ??Ӧ?ڵ??????ݱ?ʶ(?? 1-1) */
volatile struct CurrentDataBlock_1 {
	unsigned int thisCurrent;//??ǰ©????ֵ
	unsigned int currentLeakTimes;//??ǰʣ?????????޴???
	unsigned int maxCurrent;//????????©????
	unsigned int avgCurrent;//??????С©??ֵ
	unsigned int todayPowerDownTimes;//???յ???????
}CurrentDataBlock_1;

/* ʣ?????????ޱ????¼? */
volatile struct CurrentProblemRecord {
    volatile unsigned char sYear;//??ʼ??
    volatile unsigned char sMonth;//??ʼ??
    volatile unsigned char sDate;//??ʼ??
    volatile unsigned char sHour;//??ʼСʱ
    volatile unsigned char sMinute;//??ʼ????
    volatile unsigned char sSecond;//??ʼ??
    volatile unsigned int maxCurrent;//????ʣ??????ֵ
    volatile unsigned char id;//?ɼ?????id
}CurrentProblemRecord;

/* ?????¼???¼ */
struct PowerDownRecord {
    volatile unsigned char sYear;
    volatile unsigned char sMonth;
    volatile unsigned char sDate;
    volatile unsigned char sHour;
    volatile unsigned char sMinute;
    volatile unsigned char sSecond;
    volatile unsigned char id;//?ɼ?????id
};
/* ??Ӧ?????¼???¼???ݱ?ʶ????(?? 1-2)*/
struct HistoryProblem {
    unsigned char CurrentProblemTime_MSB;//ʣ???????????ܴ?????1?ֽ?
    unsigned int CurrentProblemTime_LSB;//ʣ???????????ܴ?????2?ֽ?
    struct CurrentProblemRecord currentRecord[HISTORY_CACHE_SIZE];//ʣ??????????10?μ?¼
    unsigned char currentRecordIndex;
    unsigned char VoltageProblemTime_MSB;//?ϵ??????ܴ?????1?ֽ?
    unsigned int VoltageProblemTime_LSB;//?ϵ??¹??ܴ?????2?ֽ?
    struct PowerDownRecord voltageRecord[HISTORY_CACHE_SIZE];//????????10?μ?¼
    unsigned char voltageRecordIndex;
}HistoryProblem;

/* ??Ӧ?ڲα??????ݱ?ʶ?????먱? 1-3) */
struct ParameterIdentifier {
	unsigned char CurrentThreshold;
	unsigned int VoltageUpperRange;
	unsigned int VoltageDownRange;
}ParameterIdentifier = {10,240,200};

/* ??ѹ?ϸ???????ָ??(?? 1-6) */
volatile struct OccurTime {
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char minute;
}; 

volatile struct VoltagePassRate {
    unsigned int maxVoltage;
    struct OccurTime maxVoltageOccureTime;  
    unsigned int minVoltage;
    struct OccurTime minVoltageOccureTime;
};

typedef struct AddressIdMapping {
    unsigned char address[6];//Router Address
    unsigned char id;//Router ID
    unsigned char currentIndex;//0 ~ 9
    unsigned char voltageIndex;//0 ~ 9
    unsigned char isCurrentFull;//1 or 0
    unsigned char isVoltageFull;//1 or 0
};
struct AddressIdMapping addressIdMapping[MaxRouterNumber];
//give every router 100 * 2 byte space to store history data , 100 byte for current data and 100 byte for volatge data

/* ?洢????15?յĵ?ѹ?ϸ??????? */
volatile struct VoltagePassRate voltagePassRate[15];
volatile unsigned char voltagePassRateIndex = 0;

/* Router Data Cacheing */
volatile unsigned char cache_current[CACHE_SPACE] = {0};
volatile unsigned int cache_voltage[CACHE_SPACE] = {0};
volatile unsigned char cache_ttl[CACHE_SPACE] = {0};
volatile unsigned int T0_Count = 0;
volatile unsigned int bisecondCount = 0;
volatile unsigned char modTemp = 0;

/*
Structure for read button status
*/
volatile struct {
    unsigned bit4:1;
    unsigned bit5:1;
}bitVar;
/* Button Status Code */
/*
	0 Bluetoot
	1 GPRS
	2 RS485
*/
volatile unsigned char ButtonStatus = 1;//default for GPRS

/* Configurable Parameters */
/* Coordinator Parameters Default Value */
volatile unsigned char RouterNum = 20;//Totle Router Number in a zone
volatile unsigned char QueryPeriod = 30;//30 * 50 = 1500 ms = 1.5 second(Query Period)
/* Router Parameter Default Value */
volatile unsigned char CurrentThreshold = 20;
volatile unsigned int VoltageUpperRange = 40;//(240V)upper voltage range(+5%)
volatile unsigned int VoltageDownRange = 40;//(200V)down voltage range(-10%)
volatile unsigned char MonitorVoltageID = 1;//Router that sending Voltage Monitor Data
volatile unsigned char RetransmitTimeRatio = 50;//Retransmit period Ratindex_Bluetooth++o

/* Clear Bit(Set bit to 0) */
inline unsigned char setBit(unsigned char d,unsigned char n) {
	return (d | (1<<n));
}
/* Set Bit(set bit to 1) */
/* Example : data = clearBit(data,1),means clear 1st bit of data(note data holds 1 byte) */
inline unsigned char clearBit(unsigned char d,unsigned char n) {
	return (d & (~(1<<n)));
}

/* ??ʮ??????ת????ʮ????????(12 -> 0x12 = 18) */
inline unsigned char DEC2HEX(unsigned int d) {
	unsigned char res;
	if(d > 100) return 0xFF;//????????!
	res = ((d / 10) * 16) + d % 10;
	return res;
}

inline unsigned char HEX2DEC(unsigned char d) {
    return (d / 16) * 10 + (d % 16);
}
/*
** Initialize Button and Led Pins
*/
void initIOfor485Bus() {
	DDRC |= 0x80;//LED pin - output
	DDRC &= ~(0x30);//2 Button pin - input

	/* 0011 0000 */
    DDRD |= 0x30; //make portd(4:5) output;
	/* 1101 1111 
	   we need to set and clear bit one ny one !!!!!!!
	*/
    PORTD &= 0xDF; //clear bit PD5
    PORTD |= 0x10; //set bit PD4

	/* make 485 enable pin Output pin */
	DDRB |= 0x10;//485 enable pin
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
** Read Switch Status
*/
void readButtonSatus() {
	volatile unsigned char temp;
	temp = PINC & 0x30;//button status

	ButtonStatus = ((temp & 0x10) << 1) + ((temp & 0x20) >> 1);
	ButtonStatus = ButtonStatus >> 4;

	bitVar.bit4 = temp >> 4;
	bitVar.bit5 = temp >> 5;
}
/* 
** Check Switch Status
*/
int checkStatus() {
	//if bit changed , then change the pin accordingly
	if(bitVar.bit5 ^ (PORTD & (1 << 5))) {
		if(bitVar.bit5) {
			PORTD = setBit(PORTD,5); //make output 10(connect to GPRS);
		}
	   	else {
	   		PORTD = clearBit(PORTD,5);
	   	}
	}

	//if bit changed , then change the pin accordingly
   	if(bitVar.bit4 ^ (PORTD & (1 << 4))) {
    	if(bitVar.bit4) {
    		PORTD = setBit(PORTD,4); //make output 10(connect to GPRS);
    	}
    	else {
    		PORTD = clearBit(PORTD,4);
    	}
    }

    if((~bitVar.bit4) && bitVar.bit5)return 1;
    else return 0;
}
/*
** USART0 Receive Interrupt Service Routing(485)
*/
ISR(USART0_RX_vect)//USART Receive Complete Vector
{
	unsigned char temp;	
	
	cli();

	UCSR0B &= (~(1 << RXCIE0));//disable receiver interrupt(reset bit)
	temp = UDR0;//read data
    
	if(temp == StartByte_485){
		if(startFlag_485 == 0) {
			startFlag_485 = 1;
			index_485 = 0;
		} else if(index_485 < recBufferSize_485) {
			recBuffer_485[index_485] = temp;	
			++index_485;
		}
	} else if(startFlag_485 == 1){
		if(index_485 >= recBufferSize_485 - 1) {
			startFlag_485 = 0;//bad package
			index_485 = 0;
		} else if(index_485 > 8) {//8 is the length byte index
			if(index_485 < 10 + recBuffer_485[8]) {
				recBuffer_485[index_485] = temp;
				++index_485;
			} else if (index_485 == 10 + recBuffer_485[8] && temp == EndByte_485){
				/* a valid package received */
				#ifdef DEBUG
					//WRITE485;
					//USART0_Send_Byte(0x55);
					//READ485;
				#endif
				recFlag_485 = 1;
				recNum_485 = index_485;
				startFlag_485 = 0;
				index_485 = 0;
				WRITE485;
			} else {
				#ifdef DEBUG
					WRITE485;
					USART0_Send_Byte(0x33);
					//USART0_Send_Byte(index_485);
					//USART0_Send_Byte(recBuffer_485[8]);
					//USART0_Send_Byte(recBuffer_485[index_485]);
					READ485;
				#endif
				startFlag_485 = 0;//bad package
				index_485 = 0;
			}
		} else {
			recBuffer_485[index_485] = temp;
			++index_485;
		}
	}

	UCSR0B |= (1 << RXCIE0);//re-enable receiver interrupt(set bit)
	sei();
}
/*
USART1 Receive Interrupt Service Routing(Zigbee)
*/
ISR(USART1_RX_vect)//USART Receive Complete Vector
{
	unsigned char temp;

	UCSR1B &= (~(1<<RXCIE1));//disable receiver interrupt(reset bit)
	temp = UDR1;//read data

	/* Overrun Error */
    /*if (DOR1 == 1) {
        temp = RCREG;
        temp = RCREG;
        CREN = 0;
        CREN = 1;
    }*/

    if((startFlag_Zigbee == 1) && (index_Zigbee < recBufferSize_Zigbee - 1)) {
    	recBuffer_Zigbee[index_Zigbee] = temp;
    	if(index_Zigbee == 0) {
            if(temp > 25)startFlag_Zigbee = 0;
            else recNum_Zigbee = temp;
        }
    	else if(index_Zigbee >= recNum_Zigbee){
    		if(temp == EndByte_Zigbee) {
 				recFlag_Zigbee = 1;
    		}
    		index_Zigbee = 0;
    		startFlag_Zigbee = 0;
    	}
    	++index_Zigbee;
    }

    if(temp == StartByte_Zigbee && startFlag_Zigbee == 0){
    	startFlag_Zigbee = 1;
    	index_Zigbee = 0;
    }

	UCSR1B |= (1<<RXCIE1);//re-enable receiver interrupt(set bit)
}
/*
Timer0 Service Routing
*/
ISR(TIMER0_OVF_vect)//Timer0 Overflow Interrupt Vector
{
	/* Hardware will clear Interrupt Flag for us */
	T0_Count++;
	if(T0_Count >= 624) {//about 2 second
		T0_Count = 0;

		/* feed dog every 2 second */
		__asm__ __volatile__ ("wdr");//Watch Dog Timer Reset ??

		//process down operation per 2 seconds,so we can finish all down operation in 10 seconds
		modTemp = bisecondCount % CACHE_SPACE;//Router Number
		if(cache_ttl[modTemp] != 0) {
			/* Every Time we Just Substract 100 */
			if(cache_ttl[modTemp] >= 100)cache_ttl[modTemp] = cache_ttl[modTemp] - 100;
			else cache_ttl[modTemp] = 0;
		}

		++bisecondCount;
		if (bisecondCount >= CACHE_SPACE)bisecondCount = 0;//cuz the maximum retransmit time is about 300 seconds,we set clear cache circle abot 500 second
		
		++oneMinuteCount;
		if(oneMinuteCount > 30) {// 30x2 = 60 seconds
			oneMinuteCount = 0;
			oneMinuteFlag = 1;
		}

		//TIMSK0 = (0<<TOIE0);//DISABLE TIMER 0 OVERFLOW INTERRUPT(@page 111)
	}
	TCNT0 = T0IniVal;//Timing/Counter Register
}
/*
** Get Router Id by Address
*/
unsigned char getRouterId(unsigned char *buf,unsigned char len){
    volatile unsigned char i,j;
    
    for(i = 0;i < MaxRouterNumber;++i){
        for(j = 0;j < len;++j) {
            if(addressIdMapping[i].address[j] != *(buf + j))break;
            if(j == len - 1)return (i + 1);
        }
    }

    return 255;
}
/*
** Store Received data into EEPROM
** We need to store data in different way
*/
void StoreZigbeeReceivedData() {
	unsigned int temp,i;
    unsigned char ReadTimeStatus,WriteEEPROMStatus;
	unsigned int id,Current,Voltage;
	unsigned char DataType;
	unsigned int index;

	/* Step 1: Get Router Address */
	for(i = 0;i < sizeof(myaddr);++i) {
		myaddr[i] = recBuffer_Zigbee[i + 1];//router address
	}
    
    /* Step 3: Mapping Address to ID */
    /*temp = getRouterId(myaddr,sizeof(myaddr));
    #ifdef TestUSART1
        USART0_Send_Byte(0x53);
        USART0_Send_Byte(temp);
    #endif
    if(temp == 0 || temp > MaxRouterNumber) {
        #ifdef TestUSART1
            USART0_Send_Byte(0x01);
        #endif
        return;
    }*/
    /* Step 2: Obtain Data(id , Voltage , Current , address) */
    id = recBuffer_Zigbee[sizeof(myaddr) + 1];
	Current = recBuffer_Zigbee[sizeof(myaddr) + 2] * 256 + recBuffer_Zigbee[sizeof(myaddr) + 3];
    Current = Current / 100;
	Voltage = recBuffer_Zigbee[sizeof(myaddr) + 4] * 256 + recBuffer_Zigbee[sizeof(myaddr) + 5];
    Voltage = Voltage / 100;
	DataType = recBuffer_Zigbee[sizeof(myaddr) + 6];//Zigbee_PackLen = 12
	if(id == 0 || id > MaxRouterNumber) {
        #ifdef TestUSART1
            USART0_Send_Byte(0x01);
        #endif
        return;
    } else {
        /* Store Id and Address Mapping Relationship */
        for(i = 0;i < sizeof(myaddr);++i) {
            addressIdMapping[id - 1].address[i] = myaddr[i];
        }
    }
    
    #ifdef TestUSART1
        USART0_Send_Byte(id);
        USART0_Send_Byte(Current);
        USART0_Send_Byte(Voltage);
        USART0_Send_Byte(DataType);
    #endif
	/* Step 3: Store Data into Proper Place */
    /* Abnormal Current Event */   	
	if((DataType == 0x00 && Current >= ParameterIdentifier.CurrentThreshold) || (DataType == 0x01)) {
        /* Start Location Location */
        index = EEPROM_OFFSET + (id - 1) * RouterDataUnitSize;
        if(addressIdMapping[id - 1].currentIndex > 10) {
            addressIdMapping[id - 1].currentIndex = 0;
            addressIdMapping[id - 1].isCurrentFull = 1;//now EEPROM fulls
        }
        index += addressIdMapping[id - 1].currentIndex * RouterDataItemLength;

        Read_Current_Time(DS1307,CurrentTime,7);
        /* Write Data Into EEPROM(8 byte) */
        WriteEEPROM(AT24C128,index,CurrentTime[YEAR]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[MONTH]); 
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[DATE]); 
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[HOUR]); 
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[MINUTE]); 
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[SECOND]); 
        ++index;
        WriteEEPROM(AT24C128,index,Current / 256);
        ++index;
        WriteEEPROM(AT24C128,index,Current % 256);

        /* Increment Index(for next time) */
        if(9 <= addressIdMapping[id - 1].currentIndex) {
            addressIdMapping[id - 1].currentIndex = 0;
            addressIdMapping[id - 1].isCurrentFull = 1;
        }
        else addressIdMapping[id - 1].currentIndex += 1;
	}

    /* Power Down Event */
    if(Voltage == 0){
        /* Start Location Location */
        index = EEPROM_OFFSET + (id - 1) * RouterDataUnitSize;
        index += (RouterDataUnitSize) / 2;//voltage stores in right half
        if(addressIdMapping[id - 1].voltageIndex > 10) {
            addressIdMapping[id - 1].voltageIndex = 0;
            addressIdMapping[id - 1].isVoltageFull = 1;//now EEPROM fulls
        }
        index += addressIdMapping[id - 1].voltageIndex * RouterDataItemLength;

        Read_Current_Time(DS1307,CurrentTime,7);
        WriteEEPROM(AT24C128,index,CurrentTime[YEAR]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[MONTH]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[DATE]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[HOUR]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[MINUTE]);
        ++index;
        WriteEEPROM(AT24C128,index,CurrentTime[SECOND]);

        /* Increment Index(for next time) */
        if(addressIdMapping[id - 1].voltageIndex >= 9){
            addressIdMapping[id - 1].voltageIndex = 0;
            addressIdMapping[id - 1].isVoltageFull = 1;
        }
        else addressIdMapping[id - 1].voltageIndex += 1;
    } else {
        #ifdef TestUSART1
            USART0_Send_Byte(0x02);
        #endif
    }
}
/* ??ȡʵʱ??ѹ?????????? */
unsigned int getRightNowData(volatile unsigned char *addr,unsigned char type){
    unsigned int Current = 0,Voltage = 0;
    unsigned char i;

    RealTimeQuery = 1;
    /* ??????????????,ֱ??ȡ?????????? */
    /* if there are data cached here */
    /*
    if(cache_ttl[id] > 0) {
        Current = cache_current[id];
        Voltage = cache_voltage[id];
    } else 
    */
    /* else , we just query router for data */
    /* Send Query Command to Routers */				
    USART1_Send_Byte(StartByte_Zigbee);
    USART1_Send_Byte(0x09);//package length
    for(i = 0;i < sizeof(myaddr);++i){
        USART1_Send_Byte(myaddr[i]);
    }
    USART1_Send_Byte(ZigbeeQueryByte);//Command Byte
    USART1_Send_Byte(0x00);
    USART1_Send_Byte(EndByte_Zigbee);
    /* Wait for ACK */
    for(i = 0;i < QueryPeriod;++i) {
        if(1 == recFlag_Zigbee) break;
        else _delay_ms(50);
    }
    /* Process Received Data */
    if(1 == recFlag_Zigbee) {
        recFlag_Zigbee = 0;
        /* --- Step 1: Send ACK_Zigbee to ZigBee router --- */
        /* then router stop send data to coordinator */
        for(i = 0; i < sizeof(myaddr);++i) {
            ACK_Zigbee[i + 2] = myaddr[i];
        }
        ACK_Zigbee[sizeof(myaddr) + 2] = 0x04;//command byte
        ACK_Zigbee[sizeof(myaddr) + 3] = recBuffer_Zigbee[sizeof(myaddr) + 2];//leak current high byte
        ACK_Zigbee[sizeof(myaddr) + 4] = recBuffer_Zigbee[sizeof(myaddr) + 3];//leak current low byte

        for(i = 0;i < Zigbee_AckLen;i++)
        {
            USART1_Send_Byte(ACK_Zigbee[i]);
        }

        Current = recBuffer_Zigbee[sizeof(myaddr) + 2] * 256 + recBuffer_Zigbee[sizeof(myaddr) + 3];
        if(Current < 25500) {
            Current = Current / 100;
        } else {
            Current = 255;
        }
        Voltage = recBuffer_Zigbee[sizeof(myaddr) + 4] * 256 + recBuffer_Zigbee[sizeof(myaddr) + 5];
        if(Voltage < 25500){
            Voltage = Voltage / 100;
        } else {
            Voltage = 255;
        }
    } else return MYINT_MAX;//Error Code

    RealTimeQuery = 0;

    if(type == 1) {
        return Current;
    } else if(type == 2) {
        return Voltage;
    } else return 0;//Ĭ?Ϸ???ֵ
}
/*
** general format of query data form router
** use return value or global variable to return queried data
** pointer p not used yet
*/
unsigned int generalQueryData(unsigned char command,unsigned char *buf,unsigned char len) {
	unsigned int result = 0;
	unsigned char i;

	RealTimeQuery = 1;
	/* send query command */
	USART1_Send_Byte(StartByte_Zigbee);
    USART1_Send_Byte(8 + len);//package length
    for(i = 0;i < sizeof(myaddr);++i){
    	USART1_Send_Byte(myaddr[i]);
    }
    USART1_Send_Byte(command);//Command Byte
    for(i = 0;i < len;++i){
        USART1_Send_Byte(*(buf + i));
    }
    USART1_Send_Byte(EndByte_Zigbee);
    
    /* if we were configuring parameter , we need not to wait for reply */
    if(len >= 1 && *(buf) != 0x00) return result;

    /* wait for reply data */
    for(i = 0;i < QueryPeriod;++i) {
        if(1 == recFlag_Zigbee) {
        	break;
        }
        else _delay_ms(50);
    }

    /* process received data */
    if(recFlag_Zigbee == 1) {
    	recFlag_Zigbee = 0;
    	/* store recieved data into correct place */
    	if((command == 0x05 || command == 0x08 || command == 0x09 || command == 10) && 10 == recNum_Zigbee) {
        //10 byte reply
            result = recBuffer_Zigbee[sizeof(myaddr) + 2] * 256 + recBuffer_Zigbee[sizeof(myaddr) + 3];
        } else if((command == 0x01 || command == 0x02 || command == 0x03 || command == 0x06 || command == 0x07) && 9 == recNum_Zigbee){
        //9 byte reply
            result = recBuffer_Zigbee[sizeof(myaddr) + 2];
        } else {
            #ifdef DEBUG
                USART0_Send_Byte(command);
                for(i = 0;i < len;++i) {
                    USART0_Send_Byte(*(buf + i));
                }
            #endif
        }
    } else result = MYINT_MAX;

	RealTimeQuery = 0;

	return result;
}
/*
** Reply Four 0xFE
*/
inline void ReplyTrailing4Byte() {
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
}
/* 
** Reply to Parameter Setting package 
*/
void ReplySettingParamOkPackge(unsigned char ControlByte,unsigned char *buf,unsigned char len) {
    unsigned char sum;
    unsigned char i;

    ReplyTrailing4Byte();
    sum = StartByte_485;
    USART0_Send_Byte(StartByte_485);
    for(i = 0;i < sizeof(myaddr);++i) {
        sum += myaddr[i];
        USART0_Send_Byte(myaddr[i]);
    }
    sum += StartByte_485;
    USART0_Send_Byte(StartByte_485);
    sum += ControlByte;
    USART0_Send_Byte(ControlByte);
    if(len == 0) {
        /* Normal Reply (No Error) */
        sum += 0x00;
        USART0_Send_Byte(0x00);
    } else {
        /* Abnormal Reply (Usually With 1 byte Error Code) */
        sum += len;
        USART0_Send_Byte(len);
        for(i = 0;i < len;++i) {
            sum += DecodeByte + *(buf + i);
            USART0_Send_Byte(DecodeByte + *(buf + i));
        }
    }
    USART0_Send_Byte(sum);
    USART0_Send_Byte(EndByte_485);
}
/*
** Reply with Data Acordingly
*/
void ReadDataPackage(unsigned char ControlByte){
    volatile unsigned char identify[4] = {0};
    volatile unsigned char i = 0;
    volatile unsigned char replySize = 0;
    volatile unsigned int temp,tempID,startAddr;
    volatile unsigned int Current;
    /*
     * 0x91 - no continue data(??ʱֻ????û?к???????????)
     * 0xB1 - has continue data(not finish yet)
     * ?????ֽڼ?0x80??????(?????ظ?)
     */

    /* Get idetifier - Command also*/
    for(i = 0;i < sizeof(identify);++i){
        /* ?ӵ?λ????λ???? */
        identify[i] = recData_485[sizeof(myaddr) + 3 + i] - DecodeByte;
    }
    
    /* ??һ?????????ݱ?־ */
    if(identify[3] == 0x00 && identify[2] == 0x01) {
        /* ?ظ???ǰʣ??????ֵ */
        if(identify[1] == 0x00) {
            /* Now we just Monitor a Value */
            ControlByte += 0x80;
            /* ?Ӳɼ?????ȡ??ǰ???? */
            if(identify[0] == 0x00) {
                temp = getRightNowData(myaddr,0x01);
                if(MYINT_MAX == temp) return;
                CurrentDataBlock_1.thisCurrent = temp;
            } else {
                //CurrentDataBlock_1.thisCurrent = getRightNowData(myaddr,0x01);
                return;
            }
            replyBuffer_485[0] = 0x00;//LSBС???㲿??Ĭ??Ϊ0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.thisCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.thisCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.thisCurrent / 10000);//MSB
            replySize = 4;
        }
        /* ?ظ?????ʣ?????????޴??? */
        else if(identify[1] == 0x01 && identify[0] == 0x00) {
            /* Now we just Monitor a Value */
            ControlByte += 0x80;
            /* Query Router for Results */
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x05,ZigbeeTransmitBuf,0x01);
            if(MYINT_MAX == temp) return;//no router with specific address found
            CurrentDataBlock_1.currentLeakTimes = temp;
            /* Load Data to Transmit Buffer */
            replyBuffer_485[0] = 0x00;//LSBС??????Ĭ??Ϊ0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.currentLeakTimes % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.currentLeakTimes / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.currentLeakTimes / 10000);//MSB
            replySize = 4;
        }
        /* ?ظ?????????ʣ??????ֵ */
        else if(identify[1] == 0x02 && identify[0] == 0x00) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x07,ZigbeeTransmitBuf,0x01);
            if(MYINT_MAX == temp) return;
            CurrentDataBlock_1.maxCurrent = temp;
            replyBuffer_485[0] = 0x00;//LSBС??????Ĭ??Ϊ0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.maxCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.maxCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.maxCurrent / 10000);//MSB
            replySize = 4;

        } 
        /* Today Mean Current Leakage Value */
        else if(identify[1] == 0x03 && identify[0] == 0x00) {
			/*
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSBС??????Ĭ??Ϊ0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.avgCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.avgCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.avgCurrent / 10000);//MSB
            replySize = 4;
			*/
			ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x06,ZigbeeTransmitBuf,0x01);
            if(MYINT_MAX == temp) return;
            CurrentDataBlock_1.avgCurrent = temp;
            replyBuffer_485[0] = 0x00;//LSBС??????Ĭ??Ϊ0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.avgCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.avgCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.avgCurrent / 10000);//MSB
            replySize = 4;
        }
        /* ?ظ????????????? */
        else if(identify[1] == 0x04 && identify[0] == 0x00) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSB
            /* Load Command */
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(10,ZigbeeTransmitBuf,0x01);
            if(MYINT_MAX == temp) return;
            CurrentDataBlock_1.todayPowerDownTimes = temp;
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.todayPowerDownTimes % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.todayPowerDownTimes / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.todayPowerDownTimes / 10000);//MSB
            replySize = 4;
        }
        /* ?ظ?ǰ??????????(4 * 64 = 256 byte) */
        else if(identify[1] == 0xFF && identify[0] == 0x00) {
            ControlByte += 0x80;
            /* ??ʱ??ʵ?? */
        }
    }
    
    /* ?ڶ??????????ݱ?־(??ѹ???ص?)???????˵????????͵??????? */
    if(identify[3] == 0x03){
        /* ʣ???????????ܴ???????ʱ?? */
        if(identify[2] == 0x05 && identify[1] == 0x00 && identify[0] == 0x00){
            ControlByte += 0x80;
            /* ?ܴ??? */
            temp = HistoryProblem.VoltageProblemTime_LSB;
            replyBuffer_485[0] = DEC2HEX(temp % 100);//LSB
            replyBuffer_485[1] = DEC2HEX((temp / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.VoltageProblemTime_MSB);
            /* ?????ֽ? */
            replyBuffer_485[3] = 0x00;
            replyBuffer_485[4] = 0x00;
            replyBuffer_485[5] = 0x00;//MSB
            replySize = 6;
        }
        /* ??10??ʣ?????????ޱ????¼???¼(???ݿ??Էŵ?һ???ṹ????) */
        else if (identify[2] == 0x05 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* ????identify[0]??????һ????ʮ????¼ */
                /* ??ֹʱ?? */
                /* Current Abnormal Data(last 10 times) */
                tempID = getRouterId(myaddr,sizeof(myaddr));
                if(tempID == 0 || tempID > MaxRouterNumber) {
                /* If Address is undefined */
                    #ifdef TestUSART1
                        USART0_Send_Byte(0x53);
                        USART0_Send_Byte(0x05);
                    #endif
                    return;
                }
                /* Obtain currentIndex */
                if(identify[0] > addressIdMapping[tempID - 1].currentIndex){
                    if(addressIdMapping[tempID - 1].isCurrentFull == 1) {
                        temp = HISTORY_CACHE_SIZE - (identify[0] - addressIdMapping[tempID - 1].currentIndex);
                    } else return;
                } else {
                    temp = addressIdMapping[tempID - 1].currentIndex - identify[0];
                }
                temp = (temp * RouterDataItemLength) + ((tempID - 1) * RouterDataUnitSize) + EEPROM_OFFSET;
                replyBuffer_485[5] = ReadEEPROM(AT24C128,temp);//YEAR
                replyBuffer_485[14] = replyBuffer_485[5];
                ++temp;
                replyBuffer_485[4] = ReadEEPROM(AT24C128,temp);//MONTH
                replyBuffer_485[13] = replyBuffer_485[4];
                ++temp;
                replyBuffer_485[3] = ReadEEPROM(AT24C128,temp);//DATE
                replyBuffer_485[12] = replyBuffer_485[3];
                ++temp;
                replyBuffer_485[2] = ReadEEPROM(AT24C128,temp);//HOUR
                replyBuffer_485[11] = replyBuffer_485[2];
                ++temp;
                replyBuffer_485[1] = ReadEEPROM(AT24C128,temp);//MINUTE
                replyBuffer_485[10] = replyBuffer_485[1];
                ++temp;
                replyBuffer_485[0] = ReadEEPROM(AT24C128,temp);//SECOND
                replyBuffer_485[9] = replyBuffer_485[0];
                ++temp;
                /* ©????????ֵ */
                
                Current = ReadEEPROM(AT24C128,temp);
                ++temp;
                Current = (Current * 256) + ReadEEPROM(AT24C128,temp);;
                replyBuffer_485[6] = 0x00;//
                replyBuffer_485[7] = DEC2HEX((Current % 10) * 10);//
                replyBuffer_485[8] = DEC2HEX(Current / 10);//

                replySize = 15;
            } else {
                ControlByte += 0xC0;
                return;
            }
        }
        
        /* ʣ????????????·?????ܴ??? */
        else if (identify[2] == 0x06 && identify[1] == 0x00 && identify[0] == 0x00) {
            ControlByte += 0x80;
            /* ?ܴ??? */
            temp = HistoryProblem.CurrentProblemTime_LSB;
            replyBuffer_485[0] = DEC2HEX(temp % 100);
            replyBuffer_485[1] = DEC2HEX((temp / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.CurrentProblemTime_MSB);
            /* ?????ֽ? */
            replyBuffer_485[3] = 0x00;
            replyBuffer_485[4] = 0x00;
            replyBuffer_485[5] = 0x00;

            replySize = 6;
        }
        /* ??(????)10?β?????·?????¼???¼(???ݿ??Էŵ?һ???ṹ????) */
        else if (identify[2] == 0x06 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* ????identify[0]??????һ????ʮ????¼ */
                /* ??ֹʱ?? */
                replyBuffer_485[0] = 0x00;//??
                replyBuffer_485[1] = 0x00;//??
                replyBuffer_485[2] = 0x12;//ʱ
                replyBuffer_485[3] = 0x18;//??
                replyBuffer_485[4] = 0x01;//??
                replyBuffer_485[5] = 0x16;//??
                /* ??ʼʱ?? */
                replyBuffer_485[6] = 0x00;//??
                replyBuffer_485[7] = 0x00;//??
                replyBuffer_485[8] = 0x12;//ʱ
                replyBuffer_485[9] = 0x18;//??
                replyBuffer_485[10] = 0x01;//??
                replyBuffer_485[11] = 0x16;//??

                replySize = 12;
            } else {
                ControlByte += 0xC0;
                return;
            }
        }
        /* ?????ܴ??? */
        else if (identify[2] == 0x11 && identify[1] == 0x00 && identify[0] == 0x00){
            ControlByte += 0x80;

            replyBuffer_485[0] = DEC2HEX(HistoryProblem.VoltageProblemTime_LSB % 100);
            replyBuffer_485[1] = DEC2HEX((HistoryProblem.VoltageProblemTime_LSB / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.VoltageProblemTime_MSB);
            replySize = 3;
        }
        /* ?????¼???¼ */
        else if(identify[2] == 0x11 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* ??ֹʱ?? */
                if(identify[0] > HistoryProblem.voltageRecordIndex) {
                    temp = HISTORY_CACHE_SIZE - (identify[0] - HistoryProblem.voltageRecordIndex);
                } else {
                    temp = HistoryProblem.voltageRecordIndex - identify[0];
                }
                /* Power Down Event Data(last 10 times) */
                tempID = getRouterId(myaddr,sizeof(myaddr));
                if(tempID == 0 || tempID > MaxRouterNumber) {
                /* If Address is undefined */
                    #ifdef TestUSART1
                        USART0_Send_Byte(0x04);
                    #endif
                    return;
                }
                /* Obtain voltageIndex */
                if(identify[0] > addressIdMapping[tempID - 1].voltageIndex){
                    if(addressIdMapping[tempID - 1].isVoltageFull == 1) {
                        temp = HISTORY_CACHE_SIZE - (identify[0] - addressIdMapping[tempID - 1].voltageIndex);
                    } else {
                        #ifdef TestUSART1
                            USART0_Send_Byte(addressIdMapping[tempID - 1].voltageIndex);
                            USART0_Send_Byte(0x06);
                        #endif
                        return;
                    }
                } else {
                    temp = addressIdMapping[tempID - 1].voltageIndex - identify[0];
                }
                temp = (temp * RouterDataItemLength) + ((tempID - 1) * RouterDataUnitSize) + EEPROM_OFFSET + (RouterDataUnitSize) / 2;

                replyBuffer_485[5] = ReadEEPROM(AT24C128,temp);//YEAR
                ++temp;
                replyBuffer_485[4] = ReadEEPROM(AT24C128,temp);//MONTH
                ++temp;
                replyBuffer_485[3] = ReadEEPROM(AT24C128,temp);//DATE
                ++temp;
                replyBuffer_485[2] = ReadEEPROM(AT24C128,temp);//HOUR
                ++temp;
                replyBuffer_485[1] = ReadEEPROM(AT24C128,temp);//MINUTE
                ++temp;
                replyBuffer_485[0] = ReadEEPROM(AT24C128,temp);//SECOND
                /* ??ʼʱ?? */
                replyBuffer_485[6] = replyBuffer_485[0];
                replyBuffer_485[7] = replyBuffer_485[1];
                replyBuffer_485[8] = replyBuffer_485[2];
                replyBuffer_485[9] = replyBuffer_485[3];
                replyBuffer_485[10] = replyBuffer_485[4];
                replyBuffer_485[11] = replyBuffer_485[5];

                replySize = 12;
            }
        }
        /* ??ѹ?ϸ???ͳ?????? page 42*/
        else if(identify[2] == 0x10 && (identify[1] == 0x00 || identify[1] == 0x01 || identify[1] == 0x02 || identify[1] == 0x03)) {
            /* ???????ݣ??Լ???ʮ???????? */
            if(identify[0] >= 0x00 && identify[0] <= 0x0A) {
                ControlByte += 0x80;
                if(identify[0] > voltagePassRateIndex) {
                    temp = 15 - (identify[0] - voltagePassRateIndex);
                } else {
                    temp = voltagePassRateIndex - identify[0];
                }
                /* ??ѹ????ʱ?? */
                replyBuffer_485[0] = 0x00;
                replyBuffer_485[1] = 0x32;
                replyBuffer_485[2] = 0x04;
                /* ??ѹ?ϸ??? */
                replyBuffer_485[3] = 0x00;
                replyBuffer_485[4] = 0x90;
                replyBuffer_485[5] = 0x00;
                /* ??ѹ?????? */
                replyBuffer_485[6] = 0x00;
                replyBuffer_485[7] = 0x10;
                replyBuffer_485[8] = 0x00;
                /* ??ѹ??????ʱ?? */
                replyBuffer_485[9] = 0x00;
                replyBuffer_485[10] = 0x30;
                replyBuffer_485[11] = 0x00;
                /* ??ѹ??????ʱ?? */
                replyBuffer_485[12] = 0x00;
                replyBuffer_485[13] = 0x30;
                replyBuffer_485[14] = 0x00;
                /* ???ߵ?ѹ */
                replyBuffer_485[15] = DEC2HEX((voltagePassRate[temp].maxVoltage % 10) * 10);
                replyBuffer_485[16] = DEC2HEX(voltagePassRate[temp].maxVoltage / 10);
                /* ???ߵ?ѹ????ʱ?? */
                replyBuffer_485[17] = voltagePassRate[temp].maxVoltageOccureTime.minute;//??
                replyBuffer_485[18] = voltagePassRate[temp].maxVoltageOccureTime.hour;//ʱ
                replyBuffer_485[19] = voltagePassRate[temp].maxVoltageOccureTime.date;//??
                replyBuffer_485[20] = voltagePassRate[temp].maxVoltageOccureTime.month;//??
                /* ???͵?ѹ */
                replyBuffer_485[21] = DEC2HEX((voltagePassRate[temp].minVoltage % 10) * 10);
                replyBuffer_485[22] = DEC2HEX(voltagePassRate[temp].minVoltage / 10);
                /* ???͵?ѹ????ʱ?? */
                replyBuffer_485[23] = voltagePassRate[temp].minVoltageOccureTime.minute;//??
                replyBuffer_485[24] = voltagePassRate[temp].minVoltageOccureTime.hour;//ʱ
                replyBuffer_485[25] = voltagePassRate[temp].minVoltageOccureTime.date;//??
                replyBuffer_485[26] = voltagePassRate[temp].minVoltageOccureTime.month;//??

                replySize = 27;
            }
        }
    }

    /* ?????????????ݱ?־ */
    if(identify[3] == 0x04) {
        if(identify[2] == 0x00 && identify[1] == 0x01 && identify[0] == 0x01) {
            /* ?ظ??????????? */
            ControlByte += 0x80;
            /* ??ȡ??ʱ?? */
            Read_Current_Time(DS1307,CurrentTime,7);
            replyBuffer_485[0] = CurrentTime[WEEKDAY];
            replyBuffer_485[1] = CurrentTime[DATE];
            replyBuffer_485[2] = CurrentTime[MONTH];
            replyBuffer_485[3] = CurrentTime[YEAR];
            replySize = 4;
        }
        /* ?ظ?ʱ???? */
        else if(identify[2] == 0x00 && identify[1] == 0x01 && identify[0] == 0x02) {
            ControlByte += 0x80;
            /* ??ȡ??ʱ?? */
            Read_Current_Time(DS1307,CurrentTime,7);
            replyBuffer_485[0] = CurrentTime[SECOND];
            replyBuffer_485[1] = CurrentTime[MINUTE];
            replyBuffer_485[2] = CurrentTime[HOUR];
            replySize = 3;
        }
        /* ?ظ?ͨ?ŵ?ַ */
        else if(identify[2] == 0x00 && identify[1] == 0x04 && identify[0] == 0x01) {
            ControlByte += 0x80;
            for(i = 0;i < sizeof(myaddr);++i) {
                replyBuffer_485[i] = myaddr[i];
            }
            replySize = sizeof(myaddr);
        }
        /* ?ظ?ʣ????????????ֵ */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x01) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x01,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.CurrentThreshold = temp;
            replyBuffer_485[0] = 0x00;
            replyBuffer_485[1] = 0x00;
            replyBuffer_485[2] = DEC2HEX(ParameterIdentifier.CurrentThreshold % 100);
            replySize = 3;
        }
        /* ?ظ?ʣ??????Ԥ????ֵ */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x02) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x01,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.CurrentThreshold = temp;
            replyBuffer_485[0] = 0x00;
            replyBuffer_485[1] = 0x00;
            replyBuffer_485[2] = DEC2HEX(ParameterIdentifier.CurrentThreshold % 100);
            replySize = 3;
        }
        /* ?ظ???ѹ????ֵ */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x03) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x02,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.VoltageUpperRange = temp + 220;
            //USART0_Send_Byte(ParameterIdentifier.VoltageUpperRange - 220);
            temp = ParameterIdentifier.VoltageUpperRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* ?ظ???ѹ????ֵ */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x04) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            
            temp = generalQueryData(0x03,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.VoltageDownRange = 220 - temp;
            temp = ParameterIdentifier.VoltageDownRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* ?ظ?ʣ??????????ʱ?? */
        else if(identify[2] == 0x00 && identify[1] == 0x0D && identify[0] == 0x01) {
            ControlByte += 0x80;
            /* Reply With a Default Value */
            replyBuffer_485[0] = 0x02;
            replyBuffer_485[1] = 0x00;
            replySize = 2;
        }
        /* ?ظ???ѹ???????? */
        else if(identify[2] == 0x09 && identify[1] == 0x0C && identify[0] == 0x01) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x02,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.VoltageUpperRange = temp + 220;
            temp = ParameterIdentifier.VoltageUpperRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* ?ظ???ѹ???????? */
        else if(identify[2] == 0x09 && identify[1] == 0x0C && identify[0] == 0x02) {
            ControlByte += 0x80;
            ZigbeeTransmitBuf[0] = 0x00;
            temp = generalQueryData(0x03,ZigbeeTransmitBuf,1);
            if(MYINT_MAX == temp) return;
            ParameterIdentifier.VoltageDownRange = 220 - temp;
            temp = ParameterIdentifier.VoltageDownRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
    }

    /* ?????????ݱ?־(???????ݱ?־??) */
    if(identify[3] == 0x05) {
        if(identify[2] == 0x04 && identify[1] == 0x02) {
            if(identify[0] >= 0x01 && identify[0] <= 0x08) {
                ControlByte += 0x80;
                if(monitorIndex >= (MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE / 2)) {
                    startAddr = (MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE / 2);
                } else {
                    startAddr = MONITOR_EEPROM_DOWN;
                }
                /* ???????? */
                if(identify[0] <= 4){
                /* ?????ǽ?????????Ҫ???ǳ????˵?ǰʱ???????ݣ??򷵻? */
                    if((startAddr + identify[0] * 24 * 4) > monitorIndex) {
                        return;
                    } else {
                        /* ??ǰ?????ݵ???ʵ??ַ */
                        startAddr += (identify[0] - 1) * 24 * 4;//??????ַ????Խ??
                    }
                } 
                /* ??һ?????? */
                else {
                    /* ??ǰ?????ݵ???ʵ??ַ */
                    startAddr += (identify[0] - 1) * 24 * 4 + (MONITOR_EEPROM_SIZE / 2);
                }
                /* ??ֹ???? */
                if(startAddr > MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE) startAddr -= MONITOR_EEPROM_SIZE;
                for(i = 0;i < 24;++i) {
                    temp = eeprom_read_word((uint16_t *) startAddr);//ǰ?????ֽ???©????????
                    startAddr += 4;
                    replyBuffer_485[i * 24 + 0] = 0;
                    replyBuffer_485[i * 24 + 1] = DEC2HEX(temp % 100);
                    replyBuffer_485[i * 24 + 2] = DEC2HEX((temp / 100) % 100);
                    replyBuffer_485[i * 24 + 3] = DEC2HEX(temp / 10000);
                }
                replySize = 4 * 24;
            }
        }
        else if(identify[2] == 0x06 && identify[1] == 0x00) {
            if(identify[0] >= 0x01 && identify[0] <= 0x07) {
                ControlByte += 0x80;
                
                replyBuffer_485[0] = 0x00;
                replyBuffer_485[1] = 0x00;
                replyBuffer_485[2] = 0x00;
                replyBuffer_485[3] = 0x00;
                replyBuffer_485[4] = 0x00;
                replySize = 5;
            }
        }
        else if(identify[2] == 0x06 && identify[1] == 0x01) {
            if(identify[0] >= 0x01 && identify[0] <= 0x07) {
                ControlByte += 0x80;
                
                replyBuffer_485[0] = 0x00;
                replyBuffer_485[1] = 0x00;
                replyBuffer_485[2] = 0x00;
                replyBuffer_485[3] = 0x00;
                replySize = 4;
            }
        }
    }

    /* ?????????ݱ?־ */
    if(identify[3] == 0x02) {
        /* ???ص?ѹ???? */
        if(identify[2] == 0x01 && identify[0] == 0x00) {
            if(identify[1] == 0x01 || identify[1] == 0x02 || identify[1] == 0x03) {
                ControlByte += 0x80;
                /* ???ص?ѹֵ */
                temp = getRightNowData(myaddr,0x02);
                replyBuffer_485[0] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[1] = DEC2HEX(temp / 10);
                replySize = 2;

            } else if(identify[1] == 0xFF) {
                ControlByte += 0x80;
                /* ???ص?ѹֵ???ݿ? */
                temp = getRightNowData(myaddr,0x02);
                replyBuffer_485[0] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[1] = DEC2HEX(temp / 10);
                replyBuffer_485[2] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[3] = DEC2HEX(temp / 10);
                replyBuffer_485[4] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[5] = DEC2HEX(temp / 10);
                replySize = 6;
        
            } else {
                /* ????֡???????ݰ? */
                ControlByte += 0xA0;
                /* ?????Ժ?ʵ?? */
                return;
            }
        }
    }


    /* ?ظ????ݰ? */
	/* 0xFE 0xFE 0xFE 0xFE */
	ReplyTrailing4Byte();
    /* 1. Start Byte */
    USART0_Send_Byte(StartByte_485);
    /* ??ʼ????У???? */
    CheckSum_485 = StartByte_485;
    /* 2. My Address */
    for(i = 0;i < sizeof(myaddr);++i) {
        USART0_Send_Byte(myaddr[i]);
        CheckSum_485 += myaddr[i];
    }
    /* 3. Start Byte */
    USART0_Send_Byte(StartByte_485);
    CheckSum_485 += StartByte_485;
    /* 4. ControlByte */
    USART0_Send_Byte(ControlByte);
    CheckSum_485 += ControlByte;
    /* 5. Datalength Byte */
    USART0_Send_Byte(4 + replySize);
    CheckSum_485 += 4 + replySize;
    /* 6. Identify Byte */
    for(i = 0;i < sizeof(identify);++i) {
        USART0_Send_Byte(identify[i] + DecodeByte);
        CheckSum_485 += identify[i] + DecodeByte;
    }
    /* 7. reply data Byte */
    for(i = 0;i < replySize;++i){
        USART0_Send_Byte(replyBuffer_485[i] + DecodeByte);
        CheckSum_485 += replyBuffer_485[i] + DecodeByte;
    }
    /* 8. CheckSum_485 Byte */
    USART0_Send_Byte(CheckSum_485);
    /* 9. End Byte */
    USART0_Send_Byte(EndByte_485);

    /* -- the end -- */
}
/*
** Receive Data and Write it to Memory(??Ҫ???????ò???)
*/
void WriteDataPackage(unsigned char ControlByte_485,unsigned char len) {
    volatile unsigned char identify[4] = {0};
    volatile unsigned char i = 0;
    volatile unsigned int temp,t;
    volatile unsigned char dataStartIndex = 0;
    /*
     * ?????ֽڼ?0x80??????(?????ظ?)
     */
    ControlByte_485 += 0x80;
    /* Get idetifier - Command also*/
    for(i = 0;i < sizeof(identify);++i){
        /* ?ӵ?λ????λ???? */
        identify[i] = recData_485[sizeof(myaddr) + 3 + i] - DecodeByte;
    }

    /* Get Data Start Index */
    dataStartIndex = sizeof(myaddr) + 3 + sizeof(identify);
    /* ???????? */
    for(i = dataStartIndex;i < dataStartIndex + len - sizeof(identify);++i) {
        recData_485[i] -= DecodeByte;
        //USART0_Send_Byte(recData_485[i]);
    }

    /* Validate Password */
    for(i = 0;i < sizeof(password);++i) {
        if(password[i] != recData_485[dataStartIndex + i]) {
            #ifdef DEBUG
                USART0_Send_Byte(0x03);
            #endif
            return;
        }
    }
    dataStartIndex += sizeof(password);

    /* ???????ݱ?ʶ ??1-3 */
    if(identify[3] == 0x04 && identify[2] == 0x00) {
        if(identify[1] == 0x01 && identify[0] == 0x01) {
            //InitDateTime(0x00,0x00,0x00,0x01,0x22,0x02,0x16);
            InitDate(recData_485[dataStartIndex + 3],recData_485[dataStartIndex + 2],recData_485[dataStartIndex + 1],recData_485[dataStartIndex]);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x01 && identify[0] == 0x02) {
            /* ????ʱ?? */
            InitTime(recData_485[dataStartIndex + 2],recData_485[dataStartIndex + 1],recData_485[dataStartIndex]);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x04 && identify[0] == 0x01) {
            /* ????ͨ?ŵ?ַ */
            for(i = 0; i < sizeof(myaddr);++i,dataStartIndex++) {
                myaddr[i] = recData_485[dataStartIndex];
            }
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x0E && (identify[0] == 0x01 || identify[0] == 0x02)){
            /* ???õ?????????ֵ */
            ParameterIdentifier.CurrentThreshold = HEX2DEC(recData_485[dataStartIndex + 2]);
            /* Load Data */
            ZigbeeTransmitBuf[0] = (ParameterIdentifier.CurrentThreshold) & 0xFF;
            generalQueryData(1,ZigbeeTransmitBuf,1);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x0E && identify[0] == 0x03) {
            /* ???õ?ѹ???????? */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageUpperRange = temp;
            /* Load Data */
            if((temp & 0xFF) > 220)ZigbeeTransmitBuf[0] = (temp & 0xFF) - 220;
            else {
                return;
            }
            //USART0_Send_Byte(ZigbeeTransmitBuf[0]);
            generalQueryData(2,ZigbeeTransmitBuf,1);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x0E && identify[0] == 0x04) {
            /* ???õ?ѹ???????? */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageDownRange = temp;
            /* Load Data */
            #ifdef DEBUG
                USART0_Send_Byte(0x53);
                USART0_Send_Byte(temp);
            #endif
            if((temp & 0xFF) < 220)ZigbeeTransmitBuf[0] = 220 - (temp & 0xFF);
            else return;
            generalQueryData(3,ZigbeeTransmitBuf,1);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x0C && identify[0] == 0x01){
            /* ???õ?ѹ???????? */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageUpperRange = temp;
            /* Load Data */
            if((temp & 0xFF) > 220)ZigbeeTransmitBuf[0] = (temp & 0xFF) - 220;
            else return;
            //USART0_Send_Byte(ZigbeeTransmitBuf[0]);
            generalQueryData(2,ZigbeeTransmitBuf,1);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else if(identify[1] == 0x0C && identify[0] == 0x02) {
            /* ???õ?ѹ???????? */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageDownRange = temp;
            /* Load Data */
            if((temp & 0xFF) < 220)ZigbeeTransmitBuf[0] = 220 - (temp & 0xFF);
            else return;
            generalQueryData(3,ZigbeeTransmitBuf,1);
            ReplySettingParamOkPackge(ControlByte_485,NULL,0);
        } else {
            #ifdef DEBUG
                USART0_Send_Byte(0x11);
            #endif
        }
    }
}
/* 
** Process(Store) Zigbee End Received Data
*/
void ReceivedDataProcess_485(int num) {
	unsigned int i = 0;
    unsigned char temp;
	/* Chech Start Byte */	
	if(recData_485[sizeof(myaddr)] != StartByte_485) {
		#ifdef DEBUG
		    USART0_Send_Byte(0x10);
		#endif
		return;
	}

    /* Get(Parse) Address */
    for(i = 0;i < sizeof(myaddr);++i) {
    	myaddr[i] = recData_485[i];
    }
    
    /* Router Present ?  */
    ZigbeeTransmitBuf[0] = 0x00;
    temp = generalQueryData(1,ZigbeeTransmitBuf,1);
    if(MYINT_MAX == temp){
        #ifdef DEBUG
            USART0_Send_Byte(0x11);
        #endif
        return;//Router Not Exist , Reply Nothing
    }
    /* ??ʼ????У???? */
	CheckSum_485 = StartByte_485;
	for(i = 0;i < num - 1;++i) {
		CheckSum_485 += recData_485[i];
	}
	/* Check Sum */
	if(CheckSum_485 != recData_485[num - 1]) {
		#ifdef DEBUG
            USART0_Send_Byte(0x20);
            USART0_Send_Byte(CheckSum_485);
            USART0_Send_Byte(recData_485[num - 1]);
            USART0_Send_Byte(num);
		#endif
		return;
	}
	
	/* Check Address(We don't need to check address) */

    /* ?????ֽڣ???ʶ???ݰ????? */	
	ControlByte_485 = recData_485[sizeof(myaddr) + 1];
    /* ???ݰ??????ݲ??ֳ??? */
	DataLength_485 = recData_485[sizeof(myaddr) + 2];
    /* in common case , DataLength_485 is 4 */
    
	/* Prepare Data for Different Control Byte
    ** ???ݲ?ͬ?Ŀ????ַַ?????ͬ???Ӻ???
	** start from Page 14 
    */
	switch(ControlByte_485) {
		/* Broadcast Time Calibration */
		case 0x08: {
			/* ??ʱ??ַ??ȫ0x99 */
			/* ??????????Ҫ?ظ? */

			break;
		}
		/* Read Data */
		case 0x11: {
			/* ͨ???????ݰ??ĳ?????4 */
            if(DataLength_485 != 4) {
                #ifdef DEBUG
                    USART0_Send_Byte(0x40);
                #endif
                return;
            }
            ReadDataPackage(ControlByte_485); 
            break;
		}
		/* Read Continue Data */
		case 0x12: {
			/* ?ظ???????Ϊ0x92 */
            #ifdef DEBUG
                USART0_Send_Byte(0x50);
            #endif
            break;
		}
		/* Read Address */
		case 0x13: {
			/* ??ʱ??ַ??ȫ0xAA */
			/* ?ظ???????Ϊ0x93 */
			break;
		}
		/* Write Data */
		case 0x14: {
			/* ?ظ???????Ϊ0x94 */
			WriteDataPackage(ControlByte_485,DataLength_485); 
            break;
		}
		/* Write Address */
		case 0x15: {
			/* ??ʱ??ַ??ȫ0xAA */
			/* ?ظ???????Ϊ0x95 */
			break;
		}
		default: {
            #ifdef DEBUG
                USART0_Send_Byte(0xF0);
            #endif
		}
	}
	
	#ifdef DEBUG
		USART0_Send_Byte(0x00);
	#endif
}

int main() {

    volatile unsigned char i = 0, j = 0;
    volatile unsigned char r_staus;
    volatile unsigned char t;
	volatile unsigned int temp;
    cli();

    /* Initialization */
    TWI_Init();
    USART0_Init(2400);//Initialize USART0 with baud rate of 2400(485 Bus)
    USART1_Init(38400);//Initialize USART1 with baud rate of 38400(Zigbee)
    Timer0_Init();
    InitWatchDogTimer();
    initIOfor485Bus();
    //CheckParameter();
	
    sei();            //Enable Gloabal Interrupt
    _delay_ms(50);

	#ifdef DEBUG
		WRITE485;
		USART0_Send_Byte(0x55);//for debug Watch Dog Timer
        //eeprom_update_word((uint16_t *)2068,9);
        //temp = eeprom_read_word((uint16_t *)20);
        //USART0_Send_Byte(temp / 256);
        //USART0_Send_Byte(temp % 256);
        //t = eeprom_read_byte((uint8_t *)20);
        //USART0_Send_Byte(t);
        //t = eeprom_read_byte((uint8_t *)21);
        //USART0_Send_Byte(t);
		//ReadAddr();
	#endif


	READ485;
    /* ??ʼ??????  */
    HistoryProblem.CurrentProblemTime_MSB = 0;
    HistoryProblem.CurrentProblemTime_LSB = 0;
    HistoryProblem.currentRecordIndex = 0;
    HistoryProblem.voltageRecordIndex = 0;
    voltagePassRate[voltagePassRateIndex].minVoltage = 220;
    
    while(1) {
		/* read switch button status */
		//readButtonSatus();
		//t = checkStatus();
		
		/* If Valid Data have been Received From Zigbee */
		if(1 == recFlag_Zigbee) {
            #ifdef TestUSART1
                WRITE485;
                for(i = 0;i < recNum_Zigbee;++i){
                    USART0_Send_Byte(recBuffer_Zigbee[i]);
                }
                READ485;
            #endif
		    cli();	//clear global interrupt
		    recFlag_Zigbee = 0;
		    /* --- Step 1: Send ACK_Zigbee to ZigBee router --- */
		    /* then router stop send data to coordinator */

		    /* Store Received Data to EEPROM */
		    if(recNum_Zigbee == Zigbee_PackLen) {//added 1 byte (07-15-2015)
                /* Package Formate */
                // Lenght + Address(6) + DeviceId + Current(2) + Voltage(2) + EventType
                //
                for(i = 0;i < sizeof(myaddr);++i) {
                    ACK_Zigbee[i + 2] = myaddr[i];
                }
                ACK_Zigbee[sizeof(myaddr) + 2] = 0x04;
                ACK_Zigbee[sizeof(myaddr) + 3] = recBuffer_Zigbee[sizeof(myaddr) + 2];//leak current high byte
                ACK_Zigbee[sizeof(myaddr) + 4] = recBuffer_Zigbee[sizeof(myaddr) + 3];//leak current low byte
                for(i = 0;i < Zigbee_AckLen;i++) {
                    /* Send Acknowledgement Packet to Router */
                    USART1_Send_Byte(ACK_Zigbee[i]);
                }
		    
				LEDON();
                #ifdef TestUSART1
                    WRITE485;
                #endif
				if(RealTimeQuery == 0) StoreZigbeeReceivedData();//Ignore Query Staus
				#ifdef TestUSART1
                    READ485;
                #endif
                LEDOFF();
				//USART0_Send_Byte(0x35);
				//USART0_Send_Byte(0x0A);
		    }

		    /* clear receive buffer @ 06_09 */
		    for (i = 0; i < recBufferSize_Zigbee; ++i) {
				recBuffer_Zigbee[i] = 0;
		    }

		    sei();	//set global interrupt
		}

		/* If Valid Data have been Received From Bluetooth */
		if(1 == recFlag_485) {
		    recFlag_485 = 0;

            WRITE485;

		    LEDON();
			_delay_ms(20);
		    /* Copy data from buffer to a array */
			for(j = 0;j < recNum_485;++j) {
		    	recData_485[j] = recBuffer_485[j];
		    	// Clear Data
		    	recBuffer_485[j] = 0;
		    }

		    //subtract start and end byte of received data
			//WRITE485;
			#ifdef DEBUG
			for(j = 0;j < recNum_485;++j){
	//			USART0_Send_Byte(recData_485[j]);
			}
			#endif

			ReceivedDataProcess_485(recNum_485);

			recNum_485 = 0;
			LEDOFF();

			READ485;
		}
		
		/* ÿ??1???Ӹ??µ?ǰСʱ?????ж??Ƿ????˵ڶ??? */
		if(1 == oneMinuteFlag) {
			oneMinuteFlag = 0;
			/* ??ȡСʱ?? */
			t = ReadDS1307(DS1307,HOUR);
			if(t < ThisHour) {
				/* ?Ѿ??ǵڶ????? OR ϵͳ?ո????? */
				/* ??CurrentDataBlock_1 ?еļ???λ?????? */
				CurrentDataBlock_1.currentLeakTimes = 0;
				CurrentDataBlock_1.maxCurrent = 0;
				/* ƽ??????Ŀǰ?????? */
				//CurrentDataBlock_1.avgCurrent = ;
				CurrentDataBlock_1.todayPowerDownTimes = 0;

                voltagePassRateIndex += 1;
                if(voltagePassRateIndex >= 15) voltagePassRateIndex = 0;
                voltagePassRate[voltagePassRateIndex].minVoltage = 220;
                //notify Routers
                USART1_Send_Byte(StartByte_Zigbee);
                USART1_Send_Byte(0x05);
                for(i = 0;i < 4;++i) {
                    USART1_Send_Byte(0xFF);
                }
                USART1_Send_Byte(EndByte_Zigbee);
			}
            ThisHour = t;//???µ?ǰСʱ??
		}

		//_delay_ms(1);//why delay?
    }

    return 0;
}
