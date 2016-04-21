#ifndef MYTIME_H

#define MYTIME_H
#define AcceptTimeInterval 300 // 5 minutes
/* define a timestamp variable */
typedef struct TimeStamp {
	volatile unsigned Year;// the default value(after CPU power-on) is 0
	volatile unsigned Month;
	volatile unsigned Day;
	volatile unsigned Hour;
	volatile unsigned Minute;
	volatile unsigned Second;
}TimeStamp;

unsigned char CompareTimeStamp(TimeStamp last,TimeStamp now);
unsigned int TimeInterval(TimeStamp last,TimeStamp now);

#endif
