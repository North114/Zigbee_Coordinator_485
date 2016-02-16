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

#define DEBUG

/* Zigbee Related Macro */
/** StartByte_Zigbee + UserIDByte + LeakageValueByteMSB + LeakageValueByteLSB
+ VoltageMSB + VoltagelSB + EndByte_Zigbee**/
#define recBufferSize_Zigbee 14// larger than PackLen
#define Zigbee_PackLen 7
#define Zigbee_AckLen 5

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
#define SECOND 0x00
#define MINUTE 0x01
#define HOUR 0x02
#define WEEKDAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define HISTORY_CACHE_SIZE 10
#define MAX_UCHAR 255
#define MAX_UINT 65535
#define ONCHIP_EEPROM_SIZE (2*1024) //让编译器去帮你算
#define MONITOR_EEPROM_DOWN 100  //起始地址
#define MONITOR_EEPROM_SIZE (4*24*4*2) //字节数

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

volatile unsigned char ACK_Zigbee[Zigbee_AckLen] = {StartByte_Zigbee,0,0,0,EndByte_Zigbee};


/* 485 Relatted Variable Defination
   format : startByte + address + startByte + controlByte + lengthByte + DATA + checkSum + endByte
*/
volatile unsigned char startFlag_485 = 0;
volatile unsigned char recFlag_485 = 0;//add a 'volatile' is very impportant
volatile unsigned char index_485 = 0,recNum_485 = 0;
volatile unsigned char myaddr[] = {0x12,0x34,0x56,0x78,0x90,0x12};//LSB -- MSB
volatile unsigned char recBuffer_485[recBufferSize_485];
volatile unsigned char recData_485[recBufferSize_485];
volatile unsigned char replyBuffer_485[replyBufferSize_485];

/* 帧字节 */
volatile unsigned char CheckSum_485 = 0;
volatile unsigned char ControlByte_485 = 0;
volatile unsigned char DataLength_485 = 0;

volatile unsigned char RealTimeQuery = 0;

/* 查询命令相关变量 */
volatile unsigned char ThisHour;//当前小时数
volatile unsigned char oneMinuteCount = 0;
volatile unsigned char oneMinuteFlag = 0;
volatile unsigned char monitorIndex = MONITOR_EEPROM_DOWN;

/* 对应于电能数据标识(表 1-1) */
volatile struct CurrentDataBlock_1 {
	unsigned int thisCurrent;//当前漏电流值
	unsigned int currentLeakTimes;//当前剩余电流超限次数
	unsigned int maxCurrent;//今日最大漏电流
	unsigned int avgCurrent;//今日最小漏电值
	unsigned int todayPowerDownTimes;//今日掉电次数
}CurrentDataBlock_1;

/* 剩余电流超限报警事件 */
volatile struct CurrentProblemRecord {
    volatile unsigned char sYear;//起始年
    volatile unsigned char sMonth;//起始月
    volatile unsigned char sDate;//起始日
    volatile unsigned char sHour;//起始小时
    volatile unsigned char sMinute;//起始分钟
    volatile unsigned char sSecond;//起始秒
    volatile unsigned int maxCurrent;//最大剩余电流值
    volatile unsigned char id;//采集器的id
};

/* 掉电事件记录 */
volatile struct PowerDownRecord {
    volatile unsigned char sYear;
    volatile unsigned char sMonth;
    volatile unsigned char sDate;
    volatile unsigned char sHour;
    volatile unsigned char sMinute;
    volatile unsigned char sSecond;
    volatile unsigned char id;//采集器的id
};
/* 对应与表事件记录数据标识编码(表 1-2)*/
volatile struct HistoryProblem {
    unsigned char CurrentProblemTime_MSB;//剩余电流超限总次数高1字节
    unsigned int CurrentProblemTime_LSB;//剩余电流超限总次数低2字节
    struct CurrentProblemRecord currentRecord[HISTORY_CACHE_SIZE];//剩余电流最近10次记录
    unsigned char currentRecordIndex;
    unsigned char VoltageProblemTime_MSB;//断电故障总次数高1字节
    unsigned int VoltageProblemTime_LSB;//断电事故总次数低2字节
    struct PowerDownRecord voltageRecord[HISTORY_CACHE_SIZE];//掉电最近10次记录
    unsigned char voltageRecordIndex;
}HistoryProblem;

/* 对应于参变量数据标识编码表(表 1-3) */
volatile struct ParameterIdentifier {
	unsigned char CurrentThreshold;
	unsigned char VoltageUpperRange;
	unsigned char VoltageDownRange;
}ParameterIdentifier = {10,240,200};

/* 电压合格率相关指标(表 1-6) */
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
/* 存储最近15日的电压合格率数据 */
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
volatile unsigned char CurrentThreshold = 10;
volatile unsigned char VoltageUpperRange = 40;//(240V)upper voltage range(+5%)
volatile unsigned char VoltageDownRange = 40;//(200V)down voltage range(-10%)
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

/* 将十进制数转化成十六进制数(12 -> 0x12 = 18) */
inline unsigned char DEC2HEX(unsigned int d) {
	unsigned char res;
	if(d > 100) return 0xFF;//发生错误!
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

	/* Process Received Data that comes from Zigbee */
    if((startFlag_Zigbee == 1)&&(index_Zigbee < recBufferSize_Zigbee - 1)){
     	recBuffer_Zigbee[index_Zigbee] = temp;
        index_Zigbee++;
    }
    /* here we decide weather received data are valid */
    if(temp == StartByte_Zigbee){
        startFlag_Zigbee = 1;//when we received a start byte,set startFlag
        index_Zigbee = 0;//initialize index_Zigbee,very important
    }
    else if((startFlag_Zigbee == 1)&&(temp == EndByte_Zigbee)){//endByte only make sense when startByte appeare
        startFlag_Zigbee = 0;//when we received a end byte,reset startFlag
        recNum_Zigbee = index_Zigbee;
        index_Zigbee = 0;
        recFlag_Zigbee = 1;
    }
    else{}
	//USART1_Send_Byte(temp);

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
Store Received data into EEPROM
*/
void StoreZigbeeReceivedData() {
	unsigned int temp,i;
    unsigned char ReadTimeStatus,WriteEEPROMStatus;
	unsigned int id,Current,Voltage;
	unsigned char DataType;
	
	/* Step 1: 读取ZigBee数据包中数据 */
	id = recBuffer_Zigbee[0];//router id
	Current = recBuffer_Zigbee[1] * 256 + recBuffer_Zigbee[2];
	Voltage = recBuffer_Zigbee[3] * 256 + recBuffer_Zigbee[4];
	DataType = recBuffer_Zigbee[Zigbee_PackLen - 2];//Zigbee_PackLen = 7
	
	/* Step 2: 存储故障数据到内部EEPROM(照旧) */
	if(DataType == 0x00 || DataType == 0x01) {
	    /* 故障数据或者漏电流超限数据 存到外部EEPROM */
        /* Step 2.1: Load data into W_EEprom_Array */
        for(i = 0;i < (Zigbee_PackLen - 2);i++)
        {
            W_EEprom_Array[i] = recBuffer_Zigbee[i];
            /* Then ,Clear ZigBee receive data buffer(Zigbee_Rec)*/
            recBuffer_Zigbee[i] = 0;
        }

        /* This step also read time from DS1307,and write a block data into AT24C128 */
        ReadTimeStatus = Read_Current_Time(DS1307,CurrentTime,7);
        
        /* Step 2.2: Add Time Stamp */
        W_EEprom_Array[Zigbee_PackLen - 2] = CurrentTime[6];//year
        W_EEprom_Array[Zigbee_PackLen - 1] = CurrentTime[5];//month
        W_EEprom_Array[Zigbee_PackLen] = CurrentTime[4];//date
        W_EEprom_Array[Zigbee_PackLen + 1] = CurrentTime[2];//hour
        W_EEprom_Array[Zigbee_PackLen + 2] = CurrentTime[1];//minute
        W_EEprom_Array[Zigbee_PackLen + 3] = CurrentTime[0];//second
        /* Step 2.3: Fill Currenttly Unused Part */
        W_EEprom_Array[Zigbee_PackLen + 4] = 0x23;//ASCII of '#' mark,it means the data are unread
        for(i = (Zigbee_PackLen + 5);i < BlockLength;i++){
            W_EEprom_Array[i] = 0;//the remaining data byte are set to zero
        }
        /* Step 2.4: 将数据存储到外部EEPROM */
        LastUnReadByteAddr = 256*ReadEEPROM(AT24C128,21);//High Byte of Address
        LastUnReadByteAddr += ReadEEPROM(AT24C128,22);//Low Byte of Address
        /* 将数据写入到外部EEPROM */ 
        for(i = 0;i < BlockLength;i++) {
            WriteEEPROMStatus = WriteEEPROM(AT24C128,LastUnReadByteAddr + i + 1,W_EEprom_Array[i]);
            _delay_ms(1);//delay 1 mili-second
        }
        /* 更改地址 */
        LastUnReadByteAddr += BlockLength;
        /* 处理地址向上越界 */
        if(LastUnReadByteAddr >= EEpromSize - 1) {//maximum address : EEpromSize - 1
            LastUnReadByteAddr = ReservedByteNum - 1;//pull the address back to initial address
            EEpromFull = 1;//now EEProm are full
            WriteEEPROM(AT24C128,23,EEpromFull);
        }
        /* 处理地址向下越界 */
	    else if(LastUnReadByteAddr < ReservedByteNum-1) LastUnReadByteAddr = ReservedByteNum-1;
        /* 读取外部EEPROM满了 */
        EEpromFull = ReadEEPROM(AT24C128,23);
        /* 如果满了就要修改其他地址的值了 */
        if(EEpromFull == 1) {
            // now,we entering a circle storage state.
            //	and these two address are neighbour in the following part.
            FirstReadByteAddr = LastUnReadByteAddr + 1;
            WriteEEPROM(AT24C128,15,FirstReadByteAddr>>8);//write back High Byte
            WriteEEPROM(AT24C128,16,FirstReadByteAddr&0xFF);//write back Low Byte
                    
            //	Besides,data district of read data might be invaded
            //	So,we should move the address forward accordingly,although it might corrupt
            //	data,we just have to do that to minimize our losses.
            LastReadByteAddr = 256*ReadEEPROM(AT24C128,17);
            LastReadByteAddr += ReadEEPROM(AT24C128,18);
                    
            if(FirstReadByteAddr > LastReadByteAddr + 1){
                //LastReadByteAddr
                WriteEEPROM(AT24C128,17,(FirstReadByteAddr - 1)>>8);//write back High Byte
                WriteEEPROM(AT24C128,18,(FirstReadByteAddr - 1)&0xFF);//write back Low Byte
                //FirstunReadByteAddr
                WriteEEPROM(AT24C128,19,(FirstReadByteAddr)>>8);//write back High Byte
                WriteEEPROM(AT24C128,20,(FirstReadByteAddr)&0xFF);//write back Low Byte	
            }
        }
        //	Write new LastUnReadByteAddr to EEPROM
        WriteEEPROM(AT24C128,21,LastUnReadByteAddr>>8);//write back High Byte
        WriteEEPROM(AT24C128,22,LastUnReadByteAddr&0xFF);//write back Low Byte
        /* Step 2.5: 将数据存放到相应变量 */
        if((Current / 100) >= ParameterIdentifier.CurrentThreshold) {
            /* 当日次数加一 */
            if(CurrentDataBlock_1.currentLeakTimes < 0xFFFF)
                CurrentDataBlock_1.currentLeakTimes += 1;
            /* 保留最大的漏电流值 */
            if(Current > CurrentDataBlock_1.maxCurrent){
                CurrentDataBlock_1.maxCurrent = Current / 100;
            } if(Voltage > voltagePassRate[voltagePassRateIndex].maxVoltage) {
                /* 更新当日最大电压及其发生时间 */
                voltagePassRate[voltagePassRateIndex].maxVoltage = Voltage;
                voltagePassRate[voltagePassRateIndex].maxVoltageOccureTime.month = CurrentTime[MONTH];
                voltagePassRate[voltagePassRateIndex].maxVoltageOccureTime.date = CurrentTime[DATE];
                voltagePassRate[voltagePassRateIndex].maxVoltageOccureTime.hour = CurrentTime[HOUR];
                voltagePassRate[voltagePassRateIndex].maxVoltageOccureTime.minute = CurrentTime[MINUTE];
            } if(Voltage < voltagePassRate[voltagePassRateIndex].minVoltage) {
                /* 更新当日最小电压及其发生时间 */
                voltagePassRate[voltagePassRateIndex].minVoltage = Voltage;
                voltagePassRate[voltagePassRateIndex].minVoltageOccureTime.month = CurrentTime[MONTH];
                voltagePassRate[voltagePassRateIndex].minVoltageOccureTime.date = CurrentTime[DATE];
                voltagePassRate[voltagePassRateIndex].minVoltageOccureTime.hour = CurrentTime[HOUR];
                voltagePassRate[voltagePassRateIndex].minVoltageOccureTime.minute = CurrentTime[MINUTE];
            }
            /* 总次数加1 */
            if(HistoryProblem.CurrentProblemTime_LSB >= 9999) {
                HistoryProblem.CurrentProblemTime_MSB += 1;
                HistoryProblem.CurrentProblemTime_LSB = 0;
            } else HistoryProblem.CurrentProblemTime_LSB += 1;
            /* 历史电流故障数据保存到内存中 */
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sYear = CurrentTime[YEAR];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sMonth = CurrentTime[MONTH];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sDate = CurrentTime[DATE];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sHour = CurrentTime[HOUR];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sMinute = CurrentTime[MINUTE];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].sSecond = CurrentTime[SECOND];
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].id = id;
            HistoryProblem.currentRecord[HistoryProblem.currentRecordIndex].maxCurrent = Current / 100;
            if(HistoryProblem.currentRecordIndex >= (HISTORY_CACHE_SIZE - 1)) {
                HistoryProblem.currentRecordIndex = 0;
            } else {
                HistoryProblem.currentRecordIndex += 1;
            }

        } else if((Voltage / 100) == 0) {
            /* 防止溢出 */
            if(CurrentDataBlock_1.todayPowerDownTimes < 0xFFFF) 
                CurrentDataBlock_1.todayPowerDownTimes += 1;
            /* 总次数加1 */
            if(HistoryProblem.VoltageProblemTime_LSB >= 9999) {//存的是BCD码
                HistoryProblem.VoltageProblemTime_MSB += 1;
                HistoryProblem.VoltageProblemTime_LSB = 0;
            } else HistoryProblem.VoltageProblemTime_LSB += 1;
            /* 历史电压故障数据保存到内存 */
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sYear = CurrentTime[YEAR];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sMonth = CurrentTime[MONTH];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sDate = CurrentTime[DATE];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sHour = CurrentTime[HOUR];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sMinute = CurrentTime[MINUTE];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].sSecond = CurrentTime[SECOND];
            HistoryProblem.voltageRecord[HistoryProblem.voltageRecordIndex].id = id;
            if(HistoryProblem.voltageRecordIndex >= (HISTORY_CACHE_SIZE - 1)) {
                HistoryProblem.voltageRecordIndex = 0;
            } else {
                HistoryProblem.voltageRecordIndex += 1;
            }
        }
	}
    /* 电压监测数据存储到内部EEPROM(共2kB),只存储2天的数据 */
    else if(DataType == 0x03){
        /* 前提是每15分钟一次监控数据 */
        eeprom_update_word((uint16_t *)monitorIndex,Current);
        monitorIndex += 2;
        eeprom_update_word((uint16_t *)monitorIndex,Voltage);
        monitorIndex += 2;

        if(monitorIndex >= MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE) {
            monitorIndex = MONITOR_EEPROM_DOWN;
        }

	} else {
		return;
	}

	
}
/* 
** 断电后重新上电就会读取原来的配置参数(对于目前的485总线没有作用)
*/
void CheckParameter() {
    /* address 10 is the flag byte , indicate that if we already write on-chip eeprom */
    if(eeprom_read_byte((uint8_t *)10) == 0x55) {
        /* we already configed parameters , so use these parameters */
        RouterNum = eeprom_read_byte((uint8_t *)0);
        QueryPeriod = eeprom_read_byte((uint8_t *)1);
        CurrentThreshold = eeprom_read_byte((uint8_t *)2);
        VoltageUpperRange = eeprom_read_byte((uint8_t *)3);
        VoltageDownRange = eeprom_read_byte((uint8_t *)4);
        MonitorVoltageID = eeprom_read_byte((uint8_t *)5);
        RetransmitTimeRatio = eeprom_read_byte((uint8_t *)6);
    }
}
/* 获取实时电压或电流数据 */
unsigned int getRightNowData(unsigned char type,unsigned char id){
    unsigned int Current = 0,Voltage = 0;
    unsigned char i;

    RealTimeQuery = 1;
    /* 如果缓存了数据,直接取出缓存数据 */
    if(cache_ttl[id] > 0) {
        Current = cache_current[id];
        Voltage = cache_voltage[id];
    }
    else {
        /* Query Certain Router */
        /* Send Query Command to Routers */				
        USART1_Send_Byte(StartByte_Zigbee);
        USART1_Send_Byte(id);
        USART1_Send_Byte(ZigbeeQueryByte);//Command Byte
        USART1_Send_Byte(EndByte_Zigbee);
        /* Wait for ACK */
        for(i = 0;i < QueryPeriod;++i) {
            if(1 == recFlag_Zigbee) break;
            else _delay_ms(50);
        }
        if(1 == recFlag_Zigbee) {
            recFlag_Zigbee = 0;
            /* --- Step 1: Send ACK_Zigbee to ZigBee router --- */
            /* then router stop send data to coordinator */
            ACK_Zigbee[1] = recBuffer_Zigbee[0];//router device id
            ACK_Zigbee[2] = recBuffer_Zigbee[1];//leak current high byte
            ACK_Zigbee[3] = recBuffer_Zigbee[2];//leak current low byte
            for(i = 0;i < Zigbee_AckLen;i++)
            {
                USART1_Send_Byte(ACK_Zigbee[i]);
            }

            Current = recBuffer_Zigbee[1] * 256 + recBuffer_Zigbee[2];
            Voltage = recBuffer_Zigbee[3] * 256 + recBuffer_Zigbee[4];
        }
    }

    RealTimeQuery = 0;

    if(type == 1) {
        return Current;
    } else if(type == 2) {
        return Voltage;
    } else return 0;//默认返回值
}
/*
** Reply Four 0xFE
*/
inline void ReplyTrailing4Byte(){
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
	USART0_Send_Byte(0xFE);
}

/*
** Reply with Data Acordingly
*/
void ReadDataPackage(unsigned char ControlByte){
    volatile unsigned char identify[4] = {0};
    volatile unsigned char i = 0;
    volatile unsigned char replySize = 0;
    volatile unsigned int temp,startAddr;
    /*
     * 0x91 - no continue data(暂时只考虑没有后续包的情况)
     * 0xB1 - has continue data(not finish yet)
     * 控制字节加0x80就行了(正常回复)
     */

    /* Get idetifier - Command also*/
    for(i = 0;i < sizeof(identify);++i){
        /* 从低位到高位排列 */
        identify[i] = recData_485[sizeof(myaddr) + 3 + i] - DecodeByte;
    }
    
    /* 第一种类型数据标志 */
    if(identify[3] == 0x00 && identify[2] == 0x01) {
        /* 回复当前剩余电流值 */
        if(identify[1] == 0x00) {
            /* Now we just Monitor a Value */
            ControlByte += 0x80;
            /* 从采集器获取当前电流 */
            if(identify[0] == 0x00) {
                CurrentDataBlock_1.thisCurrent = getRightNowData(0x01,MonitorVoltageID);
            } else {
                CurrentDataBlock_1.thisCurrent = getRightNowData(0x01,identify[0]);
            }
            replyBuffer_485[0] = 0x00;//LSB小数点部分默认为0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.thisCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.thisCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.thisCurrent / 10000);//MSB
            replySize = 4;
        }
        /* 回复当天剩余电流超限次数 */
        else if(identify[1] == 0x01 && identify[0] == 0x00) {
            /* Now we just Monitor a Value */
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSB小数部分默认为0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.currentLeakTimes % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.currentLeakTimes / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.currentLeakTimes / 10000);//MSB
            replySize = 4;
        }
        /* 回复当天最大剩余电流值 */
        else if(identify[1] == 0x02 && identify[0] == 0x00) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSB小数部分默认为0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.maxCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.maxCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.maxCurrent / 10000);//MSB
            replySize = 4;

        } 
        /* 回复当天平均剩余电流 -- 数据无意义*/
        else if(identify[1] == 0x03 && identify[0] == 0x00) {
			/*
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSB小数部分默认为0
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.avgCurrent % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.avgCurrent / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.avgCurrent / 10000);//MSB
            replySize = 4;
			*/
			ControlByte += 0xC0;
			return;
        }
        /* 回复当天掉点次数 */
        else if(identify[1] == 0x04 && identify[0] == 0x00) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;//LSB
            replyBuffer_485[1] = DEC2HEX(CurrentDataBlock_1.todayPowerDownTimes % 100);
            replyBuffer_485[2] = DEC2HEX((CurrentDataBlock_1.todayPowerDownTimes / 100) % 100);
            replyBuffer_485[3] = DEC2HEX(CurrentDataBlock_1.todayPowerDownTimes / 10000);//MSB
            replySize = 4;
        }
        /* 回复前面所有数据(4 * 64 = 256 byte) */
        else if(identify[1] == 0xFF && identify[0] == 0x00) {
            ControlByte += 0x80;
            /* 暂时不实现 */
        }
    }
    
    /* 第二种类型数据标志(电压相关的)，并入了第六种类型的数据了 */
    if(identify[3] == 0x03){
        /* 剩余电流超限总次数和总时间 */
        if(identify[2] == 0x05 && identify[1] == 0x00 && identify[0] == 0x00){
            ControlByte += 0x80;
            /* 总次数 */
            temp = HistoryProblem.VoltageProblemTime_LSB;
            replyBuffer_485[0] = DEC2HEX(temp % 100);//LSB
            replyBuffer_485[1] = DEC2HEX((temp / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.VoltageProblemTime_MSB);
            /* 保留字节 */
            replyBuffer_485[3] = 0x00;
            replyBuffer_485[4] = 0x00;
            replyBuffer_485[5] = 0x00;//MSB
            replySize = 6;
        }
        /* 上10次剩余电流超限报警事件记录(数据可以放到一个结构体中) */
        else if (identify[2] == 0x05 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* 根据identify[0]给出第一到第十个记录 */
                /* 终止时间 */
                if(identify[0] > HistoryProblem.currentRecordIndex) {
                    //temp 是数组的索引
                    temp = HISTORY_CACHE_SIZE - (identify[0] - HistoryProblem.currentRecordIndex);
                } else {
                    temp = HistoryProblem.currentRecordIndex - identify[0];
                }
                replyBuffer_485[0] = HistoryProblem.currentRecord[temp].sSecond;//秒
                replyBuffer_485[1] = HistoryProblem.currentRecord[temp].sMinute;//分
                replyBuffer_485[2] = HistoryProblem.currentRecord[temp].sHour;//时
                replyBuffer_485[3] = HistoryProblem.currentRecord[temp].sDate;//日
                replyBuffer_485[4] = HistoryProblem.currentRecord[temp].sMonth;//月
                replyBuffer_485[5] = HistoryProblem.currentRecord[temp].sYear;//年
                /* 漏电流最大值 */
                replyBuffer_485[6] = 0x00;//
                replyBuffer_485[7] = DEC2HEX((HistoryProblem.currentRecord[temp].maxCurrent % 10) * 10);//
                replyBuffer_485[8] = DEC2HEX(HistoryProblem.currentRecord[temp].maxCurrent / 10);//
                /* 起始时间 */
                replyBuffer_485[9] = HistoryProblem.currentRecord[temp].sSecond;//秒
                replyBuffer_485[10] = HistoryProblem.currentRecord[temp].sMinute;//分
                replyBuffer_485[11] = HistoryProblem.currentRecord[temp].sHour;//时
                replyBuffer_485[12] = HistoryProblem.currentRecord[temp].sDate;//日
                replyBuffer_485[13] = HistoryProblem.currentRecord[temp].sMonth;//月
                replyBuffer_485[14] = HistoryProblem.currentRecord[temp].sYear;//年

                replySize = 15;
            } else {
                ControlByte += 0xC0;
                return;
            }
        }
        
        /* 剩余电流采样回路短线总次数 */
        else if (identify[2] == 0x06 && identify[1] == 0x00 && identify[0] == 0x00) {
            ControlByte += 0x80;
            /* 总次数 */
            temp = HistoryProblem.CurrentProblemTime_LSB;
            replyBuffer_485[0] = DEC2HEX(temp % 100);
            replyBuffer_485[1] = DEC2HEX((temp / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.CurrentProblemTime_MSB);
            /* 保留字节 */
            replyBuffer_485[3] = 0x00;
            replyBuffer_485[4] = 0x00;
            replyBuffer_485[5] = 0x00;

            replySize = 6;
        }
        /* 上(最近)10次采样回路断线事件记录(数据可以放到一个结构体中) */
        else if (identify[2] == 0x06 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* 根据identify[0]给出第一到第十个记录 */
                /* 终止时间 */
                replyBuffer_485[0] = 0x00;//秒
                replyBuffer_485[1] = 0x00;//分
                replyBuffer_485[2] = 0x12;//时
                replyBuffer_485[3] = 0x18;//日
                replyBuffer_485[4] = 0x01;//月
                replyBuffer_485[5] = 0x16;//年
                /* 起始时间 */
                replyBuffer_485[6] = 0x00;//秒
                replyBuffer_485[7] = 0x00;//分
                replyBuffer_485[8] = 0x12;//时
                replyBuffer_485[9] = 0x18;//日
                replyBuffer_485[10] = 0x01;//月
                replyBuffer_485[11] = 0x16;//年

                replySize = 12;
            } else {
                ControlByte += 0xC0;
                return;
            }
        }
        /* 掉电总次数 */
        else if (identify[2] == 0x11 && identify[1] == 0x00 && identify[0] == 0x00){
            ControlByte += 0x80;

            replyBuffer_485[0] = DEC2HEX(HistoryProblem.VoltageProblemTime_LSB % 100);
            replyBuffer_485[1] = DEC2HEX((HistoryProblem.VoltageProblemTime_LSB / 100) % 100);
            replyBuffer_485[2] = DEC2HEX(HistoryProblem.VoltageProblemTime_MSB);
            replySize = 3;
        }
        /* 掉电事件记录 */
        else if(identify[2] == 0x11 && identify[1] == 0x00) {
            if(identify[0] <= 0x0A && identify[0] >= 0x01) {
                ControlByte += 0x80;
                /* 终止时间 */
                if(identify[0] > HistoryProblem.voltageRecordIndex) {
                    temp = HISTORY_CACHE_SIZE - (identify[0] - HistoryProblem.voltageRecordIndex);
                } else {
                    temp = HistoryProblem.voltageRecordIndex - identify[0];
                }
                replyBuffer_485[0] = HistoryProblem.voltageRecord[temp].sSecond;//秒
                replyBuffer_485[1] = HistoryProblem.voltageRecord[temp].sMinute;//分
                replyBuffer_485[2] = HistoryProblem.voltageRecord[temp].sHour;//时
                replyBuffer_485[3] = HistoryProblem.voltageRecord[temp].sDate;//日
                replyBuffer_485[4] = HistoryProblem.voltageRecord[temp].sMonth;//月
                replyBuffer_485[5] = HistoryProblem.voltageRecord[temp].sYear;//年
                /* 起始时间 */
                replyBuffer_485[6] = HistoryProblem.voltageRecord[temp].sSecond;//秒
                replyBuffer_485[7] = HistoryProblem.voltageRecord[temp].sMinute;//分
                replyBuffer_485[8] = HistoryProblem.voltageRecord[temp].sHour;//时
                replyBuffer_485[9] = HistoryProblem.voltageRecord[temp].sDate;//日
                replyBuffer_485[10] = HistoryProblem.voltageRecord[temp].sMonth;//月
                replyBuffer_485[11] = HistoryProblem.voltageRecord[temp].sYear;//年


                replySize = 12;
            }
        }
        /* 电压合格率统计数据 page 42*/
        else if(identify[2] == 0x10 && (identify[1] == 0x00 || identify[1] == 0x01 || identify[1] == 0x02 || identify[1] == 0x03)) {
            /* 本日数据，以及上十二日数据 */
            if(identify[0] >= 0x00 && identify[0] <= 0x0A) {
                ControlByte += 0x80;
                if(identify[0] > voltagePassRateIndex) {
                    temp = 15 - (identify[0] - voltagePassRateIndex);
                } else {
                    temp = voltagePassRateIndex - identify[0];
                }
                /* 电压监测时间 */
                replyBuffer_485[0] = 0x00;
                replyBuffer_485[1] = 0x32;
                replyBuffer_485[2] = 0x04;
                /* 电压合格率 */
                replyBuffer_485[3] = 0x00;
                replyBuffer_485[4] = 0x90;
                replyBuffer_485[5] = 0x00;
                /* 电压超限率 */
                replyBuffer_485[6] = 0x00;
                replyBuffer_485[7] = 0x10;
                replyBuffer_485[8] = 0x00;
                /* 电压超上限时间 */
                replyBuffer_485[9] = 0x00;
                replyBuffer_485[10] = 0x30;
                replyBuffer_485[11] = 0x00;
                /* 电压超下限时间 */
                replyBuffer_485[12] = 0x00;
                replyBuffer_485[13] = 0x30;
                replyBuffer_485[14] = 0x00;
                /* 最高电压 */
                replyBuffer_485[15] = DEC2HEX((voltagePassRate[temp].maxVoltage % 10) * 10);
                replyBuffer_485[16] = DEC2HEX(voltagePassRate[temp].maxVoltage / 10);
                /* 最高电压出现时间 */
                replyBuffer_485[17] = voltagePassRate[temp].maxVoltageOccureTime.minute;//分
                replyBuffer_485[18] = voltagePassRate[temp].maxVoltageOccureTime.hour;//时
                replyBuffer_485[19] = voltagePassRate[temp].maxVoltageOccureTime.date;//日
                replyBuffer_485[20] = voltagePassRate[temp].maxVoltageOccureTime.month;//月
                /* 最低电压 */
                replyBuffer_485[21] = DEC2HEX((voltagePassRate[temp].minVoltage % 10) * 10);
                replyBuffer_485[22] = DEC2HEX(voltagePassRate[temp].minVoltage / 10);
                /* 最低电压出现时间 */
                replyBuffer_485[23] = voltagePassRate[temp].minVoltageOccureTime.minute;//分
                replyBuffer_485[24] = voltagePassRate[temp].minVoltageOccureTime.hour;//时
                replyBuffer_485[25] = voltagePassRate[temp].minVoltageOccureTime.date;//日
                replyBuffer_485[26] = voltagePassRate[temp].minVoltageOccureTime.month;//月

                replySize = 27;
            }
        }
    }

    /* 第三种类型数据标志 */
    if(identify[3] == 0x04) {
        if(identify[2] == 0x00 && identify[1] == 0x01 && identify[0] == 0x01) {
            /* 回复年月日星期 */
            ControlByte += 0x80;
            /* 读取出时间 */
            Read_Current_Time(DS1307,CurrentTime,7);
            replyBuffer_485[0] = CurrentTime[WEEKDAY];
            USART0_Send_Byte(CurrentTime[WEEKDAY]);
            replyBuffer_485[1] = CurrentTime[DATE];
            USART0_Send_Byte(CurrentTime[DATE]);
            replyBuffer_485[2] = CurrentTime[MONTH];
            USART0_Send_Byte(CurrentTime[MONTH]);
            replyBuffer_485[3] = CurrentTime[YEAR];
            USART0_Send_Byte(CurrentTime[YEAR]);
            replySize = 4;
        }
        /* 回复时分秒 */
        else if(identify[2] == 0x00 && identify[1] == 0x01 && identify[0] == 0x02) {
            ControlByte += 0x80;
            /* 读取出时间 */
            Read_Current_Time(DS1307,CurrentTime,7);
            replyBuffer_485[0] = CurrentTime[SECOND];
            replyBuffer_485[1] = CurrentTime[MINUTE];
            replyBuffer_485[2] = CurrentTime[HOUR];
            replySize = 3;
        }
        /* 回复通信地址 */
        else if(identify[2] == 0x00 && identify[1] == 0x04 && identify[0] == 0x01) {
            ControlByte += 0x80;
            for(i = 0;i < sizeof(myaddr);++i) {
                replyBuffer_485[i] = myaddr[i];
            }
            replySize = sizeof(myaddr);
        }
        /* 回复剩余电流超限阈值 */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x01) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;
            replyBuffer_485[1] = 0x00;
            replyBuffer_485[2] = DEC2HEX(ParameterIdentifier.CurrentThreshold % 100);
            replySize = 3;
        }
        /* 回复剩余电流预警限值 */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x02) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x00;
            replyBuffer_485[1] = 0x00;
            replyBuffer_485[2] = DEC2HEX(ParameterIdentifier.CurrentThreshold % 100);
            replySize = 3;
        }
        /* 回复电压上限值 */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x03) {
            ControlByte += 0x80;
            temp = ParameterIdentifier.VoltageUpperRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* 回复电压下限值 */
        else if(identify[2] == 0x00 && identify[1] == 0x0E && identify[0] == 0x04) {
            ControlByte += 0x80;
            temp = ParameterIdentifier.VoltageDownRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* 回复剩余电流超限时常 */
        else if(identify[2] == 0x00 && identify[1] == 0x0D && identify[0] == 0x01) {
            ControlByte += 0x80;
            replyBuffer_485[0] = 0x02;
            replyBuffer_485[1] = 0x00;
            replySize = 2;
        }
        /* 回复电压考核上限 */
        else if(identify[2] == 0x09 && identify[1] == 0x0C && identify[0] == 0x01) {
            ControlByte += 0x80;
            temp = ParameterIdentifier.VoltageUpperRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
        /* 回复电压考核下限 */
        else if(identify[2] == 0x09 && identify[1] == 0x0C && identify[0] == 0x02) {
            ControlByte += 0x80;
            temp = ParameterIdentifier.VoltageDownRange;
            replyBuffer_485[0] = DEC2HEX((temp % 10) * 10);
            replyBuffer_485[1] = DEC2HEX(temp / 10);
            replySize = 2;
        }
    }

    /* 第四种数据标志(冻结数据标志表) */
    if(identify[3] == 0x05) {
        if(identify[2] == 0x04 && identify[1] == 0x02) {
            if(identify[0] >= 0x01 && identify[0] <= 0x08) {
                ControlByte += 0x80;
                if(monitorIndex >= (MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE / 2)) {
                    startAddr = (MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE / 2);
                } else {
                    startAddr = MONITOR_EEPROM_DOWN;
                }
                /* 当日数据 */
                if(identify[0] <= 4){
                /* 如果是今日数据且要的是超过了当前时间的数据，则返回 */
                    if((startAddr + identify[0] * 24 * 4) > monitorIndex) {
                        return;
                    } else {
                        /* 当前块数据的其实地址 */
                        startAddr += (identify[0] - 1) * 24 * 4;//这个地址不会越界
                    }
                } 
                /* 上一日数据 */
                else {
                    /* 当前块数据的其实地址 */
                    startAddr += (identify[0] - 1) * 24 * 4 + (MONITOR_EEPROM_SIZE / 2);
                }
                /* 防止溢出 */
                if(startAddr > MONITOR_EEPROM_DOWN + MONITOR_EEPROM_SIZE) startAddr -= MONITOR_EEPROM_SIZE;
                for(i = 0;i < 24;++i) {
                    temp = eeprom_read_word((uint16_t *) startAddr);//前两个字节是漏电流数据
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

    /* 第五种数据标志 */
    if(identify[3] == 0x02) {
        /* 返回电压数据 */
        if(identify[2] == 0x01 && identify[0] == 0x00) {
            if(identify[1] == 0x01 || identify[1] == 0x02 || identify[1] == 0x03) {
                ControlByte += 0x80;
                /* 返回电压值 */
                temp = getRightNowData(0x02,MonitorVoltageID);
                replyBuffer_485[0] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[1] = DEC2HEX(temp / 10);
                replySize = 2;

            } else if(identify[1] == 0xFF) {
                ControlByte += 0x80;
                /* 返回电压值数据块 */
                temp = getRightNowData(0x02,MonitorVoltageID);
                replyBuffer_485[0] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[1] = DEC2HEX(temp / 10);
                replyBuffer_485[2] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[3] = DEC2HEX(temp / 10);
                replyBuffer_485[4] = DEC2HEX((temp % 10 ) * 10);
                replyBuffer_485[5] = DEC2HEX(temp / 10);
                replySize = 6;
        
            } else {
                /* 返回帧错误数据包 */
                ControlByte += 0xA0;
                /* 留待以后实现 */
                return;
            }
        }
    }


    /* 回复数据包 */
	/* 0xFE 0xFE 0xFE 0xFE */
	ReplyTrailing4Byte();
    /* 1. Start Byte */
    USART0_Send_Byte(StartByte_485);
    /* 开始计算校验和 */
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
** Receive Data and Write it to Memory(主要用于配置参数)
*/
void WriteDataPackage(unsigned char ControlByte_485,unsigned char len) {
    volatile unsigned char identify[4] = {0};
    volatile unsigned char i = 0;
    volatile unsigned int temp,t;
    volatile unsigned char dataStartIndex = 0;
    /*
     * 控制字节加0x80就行了(正常回复)
     */

    /* Get idetifier - Command also*/
    for(i = 0;i < sizeof(identify);++i){
        /* 从低位到高位排列 */
        identify[i] = recData_485[sizeof(myaddr) + 3 + i] - DecodeByte;
    }
     
    dataStartIndex = sizeof(myaddr) + 3 + sizeof(identify);
    /* 解码数据 */
    for(i = dataStartIndex;i < dataStartIndex + len - sizeof(identify);++i) {
        recData_485[i] -= DecodeByte;
        //USART0_Send_Byte(recData_485[i]);
    }

    /* 参数数据标识 表1-3 */
    if(identify[3] == 0x04 && identify[2] == 0x00) {
        if(identify[1] == 0x01 && identify[0] == 0x01) {
            /* 设置日期 */
            //InitTime(0x00,0x00,0x00,0x01,0x22,0x02,0x16);
            
            t = WriteEEPROM(DS1307,WEEKDAY,recData_485[dataStartIndex]);
            dataStartIndex += 1;
            t = WriteEEPROM(DS1307,DATE,recData_485[dataStartIndex]);
            dataStartIndex += 1;
            t = WriteEEPROM(DS1307,MONTH,recData_485[dataStartIndex]);
            dataStartIndex += 1;
            t = WriteEEPROM(DS1307,YEAR,recData_485[dataStartIndex]);
            
        } else if(identify[1] == 0x01 && identify[0] == 0x02) {
            /* 设置时间 */
            WriteEEPROM(DS1307,SECOND,recData_485[dataStartIndex]);
            dataStartIndex += 1;
            WriteEEPROM(DS1307,MINUTE,recData_485[dataStartIndex]);
            dataStartIndex += 1;
            WriteEEPROM(DS1307,HOUR,recData_485[dataStartIndex]);
        } else if(identify[1] == 0x04&& identify[0] == 0x01) {
            /* 设置通信地址 */
            for(i = 0; i < sizeof(myaddr);++i,dataStartIndex++) {
                myaddr[i] = recData_485[dataStartIndex];
            }
        } else if(identify[1] == 0x0E && (identify[0] == 0x01 || identify[0] == 0x02)){
            /* 设置电流超限阈值 */
            ParameterIdentifier.CurrentThreshold = HEX2DEC(recData_485[dataStartIndex + 2]);
        } else if(identify[1] == 0x0E && identify[0] == 0x03) {
            /* 设置电压超限上限 */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageUpperRange = temp;
        } else if(identify[1] == 0x0E && identify[0] == 0x04) {
            /* 设置电压超限下限 */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageDownRange = temp;
        } else if(identify[1] == 0x0C && identify[0] == 0x01){
            /* 设置电压超限上限 */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageUpperRange = temp;
        } else if(identify[1] == 0x0C && identify[0] == 0x02) {
            /* 设置电压超限下限 */
            temp = HEX2DEC(recData_485[dataStartIndex]) / 10;
            dataStartIndex += 1;
            temp += HEX2DEC(recData_485[dataStartIndex]) * 10;
            ParameterIdentifier.VoltageDownRange = temp;
        } else {
            #ifdef DEBUG
                USART0_Send_Byte(0x11);
            #endif
        }
    }
}
/* 
** Request Dispatcher
*/
void ReceivedDataProcess_485(int num) {
	int i = 0;
	/* Chech Start Byte */	
	if(recData_485[sizeof(myaddr)] != StartByte_485) {
		#ifdef DEBUG
		USART0_Send_Byte(0x10);
		#endif
		return;
	}
    /* 开始计算校验和 */
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
	
	/* Check Address */
	for(i = 0;i < sizeof(myaddr);++i) {
		if(myaddr[i] != recData_485[i] && 0x99 != recData_485[i] && 0xAA != recData_485[i]) {
			#ifdef DEBUG
			USART0_Send_Byte(0x30);
			#endif
			return;
		}
	}
    /* 控制字节，标识数据包类型 */	
	ControlByte_485 = recData_485[sizeof(myaddr) + 1];
    /* 数据包的数据部分长度 */
	DataLength_485 = recData_485[sizeof(myaddr) + 2];
    /* in common case , DataLength_485 is 4 */
    
	/* Prepare Data for Different Control Byte
    ** 根据不同的控制字分发到不同的子函数
	** start from Page 14 
    */
	switch(ControlByte_485) {
		/* Broadcast Time Calibration */
		case 0x08: {
			/* 此时地址是全0x99 */
			/* 这个包不需要回复 */

			break;
		}
		/* Read Data */
		case 0x11: {
			/* 通常，数据包的长度是4 */
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
			/* 回复控制码为0x92 */
            #ifdef DEBUG
                USART0_Send_Byte(0x50);
            #endif
            break;
		}
		/* Read Address */
		case 0x13: {
			/* 此时地址是全0xAA */
			/* 回复控制码为0x93 */
			break;
		}
		/* Write Data */
		case 0x14: {
			/* 回复控制码为0x94 */
			WriteDataPackage(ControlByte_485,DataLength_485); 
            break;
		}
		/* Write Address */
		case 0x15: {
			/* 此时地址是全0xAA */
			/* 回复控制码为0x95 */
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
    CheckParameter();
	
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
    /* 初始化变量  */
    HistoryProblem.CurrentProblemTime_MSB = 0;
    HistoryProblem.CurrentProblemTime_LSB = 0;
    HistoryProblem.currentRecordIndex = 0;
    HistoryProblem.voltageRecordIndex = 0;
    voltagePassRate[voltagePassRateIndex].minVoltage = 220;
    
    //InitTime(0x00,0x20,0x17,0x04,0x28,0x01,0x16);

    while(1) {
		/* read switch button status */
		//readButtonSatus();
		//t = checkStatus();
		
		/* If Valid Data have been Received From Zigbee */
		if(1 == recFlag_Zigbee) {
		    cli();	//clear global interrupt
		    recFlag_Zigbee = 0;
		    /* --- Step 1: Send ACK_Zigbee to ZigBee router --- */
		    /* then router stop send data to coordinator */
		    ACK_Zigbee[1] = recBuffer_Zigbee[0];//router device id
		    ACK_Zigbee[2] = recBuffer_Zigbee[1];//leak current high byte
		    ACK_Zigbee[3] = recBuffer_Zigbee[2];//leak current low byte
		    for(i = 0;i < Zigbee_AckLen;i++) {
		    	/* Send Acknowledgement Packet to Router */
				USART1_Send_Byte(ACK_Zigbee[i]);
		    }
		    /* Store Received Data to EEPROM */
		    if(recNum_Zigbee == (Zigbee_PackLen - 1)) {//added 1 byte (07-15-2015)
		    
				LEDON();
				if(RealTimeQuery == 0) StoreZigbeeReceivedData();//Ignore Query Staus
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
		
		/* 每隔1分钟更新当前小时数，判断是否到了第二天 */
		if(1 == oneMinuteFlag) {
			oneMinuteFlag = 0;
			/* 读取小时数 */
			t = ReadDS1307(DS1307,HOUR);
			if(t < ThisHour) {
				/* 已经是第二天了 OR 系统刚刚启动 */
				ThisHour = t;//更新当前小时数
				/* 将CurrentDataBlock_1 中的计数位都清零 */
				CurrentDataBlock_1.currentLeakTimes = 0;
				CurrentDataBlock_1.maxCurrent = 0;
				/* 平均电流目前无意义 */
				//CurrentDataBlock_1.avgCurrent = ;
				CurrentDataBlock_1.todayPowerDownTimes = 0;

                voltagePassRateIndex += 1;
                if(voltagePassRateIndex >= 15) voltagePassRateIndex = 0;
                voltagePassRate[voltagePassRateIndex].minVoltage = 220;
			}
		}

		//_delay_ms(1);//why delay?
    }

    return 0;
}
