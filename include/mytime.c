#include "mytime.h"

/* Compare 2 TimeStamp */
unsigned char CompareTimeStamp(TimeStamp last,TimeStamp now) {
    int temp;
    // if last is the default value
    if(last.Year == 0 && last.Month == 0 && last.Day == 0 && last.Hour == 0 && last.Minute == 0 && last.Second == 0) return 1;
    // compare two timestamp
    if(now.Year > last.Year) return 1;
    else if(now.Year < last.Year) return 0;
    else {
        // year is equal
        if(now.Month > last.Month) return 1;
        else if(now.Month < last.Month) return 0;
        else {
            // month is eauql
            if(now.Day > last.Day) return 1;
            else if(now.Day < last.Day) return 0;
            else {
                temp = TimeInterval(last,now);
                if(temp >= AcceptTimeInterval) return 1;
                else return 0;
            }
        }
    }
}
/* return the "time-interval(in seconds)" of 2 time*/
unsigned int TimeInterval(TimeStamp last,TimeStamp now) {
    /* Special Condition Process */
    if(now.Hour < last.Hour) return 0;
    if(now.Hour - 2 > last.Hour) return 65535;

    /* here starts functionality codes */
    int lastseconds = 0;
    int nowseconds = 0;

    lastseconds = last.Minute * 60 + last.Second;

    nowseconds = (now.Hour - last.Hour) * 3600;
    nowseconds += (now.Minute) * 60 + now.Second;

    //printf("%d\n",nowseconds);

    if(nowseconds >= lastseconds) return (nowseconds - lastseconds);
    else return 0;
}
