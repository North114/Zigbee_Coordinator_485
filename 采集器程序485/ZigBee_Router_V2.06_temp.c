/* ZigBee Router **
**Bug Sumarry:
**1.Comparator Interrupt Frequently and Unreasonably,the reasons may be switch ADC channel
**to CurruntInPin and Genarate a Narror Pulse,the Comparator might be Sensitive,so it generate
**a Comparator interrupt;(05-28-2015)
**
**
*/
/*
Change Log:
1.fitting voltge with a exponential equaltion a*(1-exp(-t/b))+c,we need to record 3 value,
that initial voltage value(V_ini) , Current Time(t_curr) , Current Voltage(V_curr),
the quation is V_real = (V_curr - V_ini)/(1 - exp(-t_curr / b)) + V_ini , so the prerequsite
of this equation is parameter b keep unchanging for different input voltage@2015-06-04.
2. 1st July,add reset for every 18 hours
----- Unknown Date

Change Log:
1.include a parameter header file --- "parameter.h" , which contains parameter of every acquier
2.change current detect stragedy , our acquier just put before protection switch , so we have no need
to detect voltage when a current rising edge occurs
3.increase voltage acquire period time to 5 to get a smooth result
4.set monitor data period to 0.5 hour - 30 minutes
5.change currentThreahold to 10mA(from 15mA)
----- 2015-11-28
*/
#include <htc.h>
#include <assert.h>
#include "parameter.h" //file contains fitting parameters

/*  @page-44*/
__CONFIG(FOSC_INTOSC & WDTE_SWDTEN & PWRTE_OFF);    //Program Config. Word 1
__CONFIG(0x2FFF);                                   //Program Config. Word 2

/* the following code are added for delay a period */
/* there are some serious problem in delay time accuarcy */
#define _XTAL_FREQ     16000000  /*Fosc 16MHZ*/
#pragma inline(_delay)
extern void _delay(unsigned long);
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000.0)))
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))

/* Macros */

#define DACThreashold 2

//#define MonitorVoltageID 0x01
#define startByte 0xAA //End byte of ZigBee data package
#define endByte 0x75 //Start byte of ZigBee data package
#define BroadcastByte 0xFF //Broadcast data byte

//#define CurrentThreshold (unsigned int)1000//(Changed for Debug)Current Leakage threshold(15mA)
#define StandardVoltage (unsigned int)22000//Acquired value when power supply is 220V AC
//#define VoltageUpperRange (unsigned int)4000//(240V)upper voltage range(+5%)
//#define VoltageDownRange (unsigned int)4000//(200V)down voltage range(-10%)
#define CurrentFlux 200

#define recBufferSize 15 //Maximum receive data length

#ifdef DEBUG
    #define MaxTransmitRetryNum 1 //define re-transmit maximum times
#else
    #define MaxTransmitRetryNum 9 //define re-transmit maximum times
#endif

#define Timer0ExpireNum (unsigned int)29412   //(0.01632*29412 = )abouts 8 Minutes(Forbidden Send Voltage Period)
//#define Timer1ExpireNum 100           //(0.05*100 = 5)about 5 second(Forbidden Send Current Period)
#define Timer2ExpireNum 200             //(0.0512*200 = 10.24)about 10 seconds(Re-Transmit Period)
#define Timer4ExpireNum 600              //(0.0512*600 = 30)about 30 Seconds(Measurement Period)
#define Timer6ExpireNum 1172            //(0.0512*1172 = 60.0064)about 1 minute(Countting Seconds)

#define AcquiredSampleNum 28 //ADC Sample Number(28 means about 20 milisecond)
//smooth for voltage acquire
#define SmoothPeriod 6

#define JumpAway(){\
    MeanCurrentValue = 0;\
    MeanVoltageValue = 22000;\
}

/* Function Declareation */
void initOsc();
void initUSART();
void initCurrentInPin();
void initVoltageInPin();
void initDAC();
void initCamparator();
void initTimer0();
void initTimer1();
void initTimer2();
void initTimer4();
void initTimer6();
void initWTD();

unsigned int ADC(unsigned char PinSelect);
void FindMaxValue();
void CurrentVoltageAcquire(unsigned char CycleNum);
void TransmitData(unsigned char Type);
void ReceivedDataProcess();
void SettingCurrentReTransmitTimes();
unsigned int getMiddleValue(unsigned char len);
int isAddressMatch(unsigned char *p,int startIndex);
void TransmitParameterData(unsigned char command,unsigned char *buf,unsigned char len);
void CheckParameters();
void eeprom_write_byte(unsigned char addr,unsigned char data);
unsigned char eeprom_read_byte(unsigned char addr);

/* Variables Declaration */
volatile unsigned char recBuffer[recBufferSize]; //10 bytes serial port receive buffer
volatile unsigned char recNum = 0; //number of received data in byte(USART),not include start byte and end byte
volatile unsigned char index = 0; //recBuffer index(USART)

volatile unsigned int T0countnum = 0; //Timer0 count number
volatile unsigned int T1countnum = 0; //Timer1 count number
volatile unsigned int T2countnum = 0; //Timer2 count number
volatile unsigned int T4countnum = 0; //Timer4 count number
volatile unsigned int T6countnum = 0; //Timer6 count number
volatile unsigned char WatchTimerDogCountnum = 0;//Watch Timer Dog count number
volatile unsigned int SystemTime = 0; //Count system time(minutes) form startup

volatile bit startFlag = 0; // 1 means received startByte data from serial port
volatile bit recFlag = 0; //Receive Data(from USART) Flag
volatile bit ackFlag = 0; //Transmit(through ZigBee) Acknowledgement Flag
volatile bit waitFlag = 0; //Actually , we can call it re-transmit-Flag
volatile bit ForbiddenSendVoltageFlag = 0; //forbidden sending voltage data through USART,notice that this flag can't forbidden current leakage data
volatile bit ForbiddenSendCurrentFlag = 0; //forbidden sending current data for certain period

//unsigned int CurrentValue[AcquiredSampleNum]; //store ADC data into gloable variable
volatile unsigned char a[2] = {0, 0}, b[2] = {0, 0}; //a matrix represent CurrentValue,b matrix represent VoltageValue
volatile unsigned int MeanCurrentValue = 0;
volatile unsigned int MeanVoltageValue = 0;
volatile unsigned int CurrentPeakValue = 0;
volatile unsigned int VoltagePeakValue = 0;
volatile unsigned int CurrentValue[AcquiredSampleNum]; //store ADC data into gloable variable

volatile unsigned int LastTimeCurrent = 0;

volatile unsigned char reTransmitTimes = 0;

volatile bit CurrentAbnormalFlag = 0;
volatile bit TimeToAcauireFlag = 0;
volatile bit resetFlag;//indicate whether the machine is reseted or start up at the first time
volatile bit CurrentContinueAbnormalFlag = 0;

volatile float float_temp;

volatile unsigned int MeasurementCurrentPeriod = 1200;//(0.05*1200 = 60)(Forbidden Send Current Period)
volatile unsigned char CurrentContinueAbnormalCount;

volatile unsigned char DataFrameType = 0x00;
/* store 5 cycle voltage value */
volatile unsigned int VoltageSmoothArray[SmoothPeriod];
/*  DataFrameType define:
**  0x00 abnormal data frame (current value or voltage value)
**  0x01 current leakage data frame(which cause powerdown of user-end)
**  0x02 real-time echo frame
**  0x03 voltage monitor data frame
*/
volatile bit MonitorVoltageFlag = 0;

/* Configurable Parameters */
volatile unsigned int CurrentThreshold = 2000;
volatile unsigned int VoltageUpperRange = 4000;//(240V)upper voltage range(+5%)
volatile unsigned int VoltageDownRange = 4000;//(200V)down voltage range(-10%)
volatile unsigned char MonitorVoltageID = 1;
volatile unsigned char RetransmitTimeRatio = 50;

volatile unsigned int Timer2ExpireTime = Timer2ExpireNum + (DeviceID * 10) % Timer2ExpireNum;

/* 485 Bus Related Variable */
//volatile unsigned char addr[] = {0x12,0x34,0x56,0x78,0x90,0x12};//Router Address
volatile unsigned int TodayCurrentExceedTime = 0;
volatile unsigned int TodayPowerDownTime = 0;
volatile unsigned int TodayMaxCurrent = 0;
volatile unsigned int TodayMeanCurrent = 0;
volatile unsigned int TodayMinVoltage = 24000;
volatile unsigned int TodayMaxVoltage = 0;
volatile unsigned char TransmitParameterBuf[2];
volatile unsigned char QuaterFlag = 0;
volatile unsigned char MinuteFlag = 0;

/* Main Function */
int main() {

    resetFlag = nRI;//record the reset flag
    nRI = 1;//set bit(but i don't know it works or not ^_^ !)

    initOsc();
    initUSART();
    initVoltageInPin();
    initCurrentInPin();
    initDAC();
    initCamparator();
    initTimer0();
    initTimer1();
    initTimer2();
    initTimer4();
    /* watch timer dog are reserved */
    initTimer6();
    initWTD();

    GIE = 1; //Global Interrupt Enable bit

    #ifdef DEBUG
        TXREG = 0X49;
        while(!TRMT);
    #endif

    /* Main Loop */
    while (1) {
        /* Re-enable Comparator 1 Interrupt */
        if (CurrentAbnormalFlag == 1) {
            /* Reset Flag */
            CurrentAbnormalFlag = 0;
            /* Acquire Current data Immidiately */
            CurrentVoltageAcquire(4);//acquire 4 cycle
            SettingCurrentReTransmitTimes();

            //if(MeanVoltageValue < 150) {
            //        #ifdef DEBUG
            //            TXREG = 0x37;
            //            while (!TRMT);
            //        #endif
            //        DataFrameType = 0x01;
            //        TransmitData(DataFrameType);
            //        /* Clear Count Number */
            //        T1countnum = 0;
            //       T0countnum = 0;
            //        /* Enable Timer1 Overflow Interrupt */
            //        TMR1IE = 1;
            //        T0IE = 1;
            //        /* Forbidden for Acquire Current Data */
            //        ForbiddenSendCurrentFlag = 1;
            //        ForbiddenSendVoltageFlag = 1;
            //} else 
            
            if ( MeanCurrentValue > CurrentThreshold ) {
            /* if current larger than 20mA , and Voltage  is close to 0 (less than 50V AC), we transmit data imediately*/
                if ( (ackFlag == 0) && (ForbiddenSendCurrentFlag == 0 || MeanCurrentValue > (LastTimeCurrent + CurrentFlux)) ) {
                    #ifdef DEBUG
                        TXREG = 0x35;
                        while (!TRMT);
                    #endif
                    DataFrameType = 0x01;//means abnormal data
                    TransmitData(DataFrameType);
                    /* Clear Count Number */
                    T1countnum = 0;
                    T0countnum = 0;
                    /* Enable Timer1 Overflow Interrupt */
                    TMR1IE = 1;
                    T0IE = 1;
                    /* Forbidden for Acquire Current Data */
                    ForbiddenSendCurrentFlag = 1;
                    ForbiddenSendVoltageFlag = 1;
                    if(TodayCurrentExceedTime < 65535)TodayCurrentExceedTime++;
                }
            } else {
                //just for debug
                #ifdef DEBUG
                    TXREG = 0x66;
                    while (!TRMT);
                #endif
            }
            //set Flag to indeicate a recent Comparator Interrupt Happend
            CurrentContinueAbnormalCount = 0;//unsigned char
            CurrentContinueAbnormalFlag = 1;//bit
            /* Clear Flag(Very Important) */
            C1IF = 0;
            /* Re-Enable Comparator Interrupt */
            C1IE = 1;
        }

        //if we have received some data from serial port,and data length is "recNum".
        if (recFlag == 1) {
            /* Reset recFlag */
            recFlag = 0;
            /* Received data Processing */
            ReceivedDataProcess();
        }

        /* If the Coordinator have not received our data ,we need to re-transmit the data */
        if ( (waitFlag == 1) && (ackFlag == 1) ) {
            /* protect un-transmitted data from Overwrite */
            /* waiting for acknowledgement */
            /* just retransmit old data */
            #ifdef DEBUG
                TXREG = 0x34;
                while (!TRMT);
            #endif

            //before ACK finished ,a[] and b[] array remain unchanged
            if (reTransmitTimes >= MaxTransmitRetryNum) {
                reTransmitTimes = 0;
                T2countnum = 0;
                TMR2IE = 0;
                waitFlag = 0;
                ackFlag = 0;
            } else {
                /* keep DataFrameType Unchanged */
                TransmitData(DataFrameType); //retransmit data to Coordinator
                ++reTransmitTimes;
            }
        }

        /* If current or voltage value larger than threshold,transmit the data through ZigBee */
        /* The precondition is there are no un-acknowledgement data */
        if ( (TimeToAcauireFlag == 1) && (ackFlag == 0) && (CurrentContinueAbnormalFlag == 0) ) {//remind : what you can do if no acknowledgement are received ?
            /* Current Leakage is high priority event */
            /* Because in real world application ,Current Leakage last for a very short period of time */
            if (0 == ForbiddenSendCurrentFlag) {
                #ifdef DEBUG
                    TXREG = 0x58;
                    while(!TRMT);
                #endif
                CurrentVoltageAcquire(1);
                SettingCurrentReTransmitTimes();
                if ((MeanCurrentValue >= CurrentThreshold) && (CurrentAbnormalFlag == 0)) {

                    #ifdef DEBUG
                        TXREG = 0x38;
                        while(!TRMT);
                    #endif
                    DataFrameType = 0x00;
                    TransmitData(DataFrameType);
                    /* Clear Count Number */
                    T1countnum = 0;
                    /* Enable Timer1 Overflow Interrupt */
                    TMR1IE = 1;
                    /* Forbidden for Acquire Current Data */
                    ForbiddenSendCurrentFlag = 1;
                    if(TodayCurrentExceedTime < 65535)TodayCurrentExceedTime++;
                }
            } else if ((ForbiddenSendVoltageFlag == 0) && (CurrentAbnormalFlag == 0)) {
                    #ifdef DEBUG
                    TXREG = 0x52;
                    while(!TRMT);
                    #endif
                CurrentVoltageAcquire(1);
                SettingCurrentReTransmitTimes();

                #ifdef DEBUG
                    TXREG = 0x32;
                    while(!TRMT);
                #endif

                if ((MeanVoltageValue > (StandardVoltage + VoltageUpperRange)) || (MeanVoltageValue < (StandardVoltage - VoltageDownRange))) {
                    DataFrameType = 0x00;
                    TransmitData(DataFrameType);
                    /* Clear Count Number */
                    T0countnum = 0;
                    /* Enable Timer0(start to counting for disable sending abnormal voltage data) */
                    T0IE = 1;
                    /* Forbidden for Acquire Voltage Data */
                    ForbiddenSendVoltageFlag = 1;
                    if(TodayPowerDownTime < 65535)TodayPowerDownTime++;
                }
            }
            /* Measuring CurrentInPin and VoltageInPin Periodicaly */
            TimeToAcauireFlag = 0;
            TMR4IE = 1;
        }

        /* Optional Compile Part(Only Compile for Device whos ID is MonitorVoltageId) */
        if ( (DeviceID == MonitorVoltageID) && (1 == MonitorVoltageFlag) && (ackFlag == 0) ) {
            MonitorVoltageFlag = 0;
            /* Acquire 1 Period of Current Value and Voltage Value */
            if (CurrentContinueAbnormalFlag == 0) {
                CurrentVoltageAcquire(1);
            }
            /* Send Data to Coordinator */
            DataFrameType = 0x03;
            TransmitData(DataFrameType);
        }

        /* Store Current and Volatage into On-chip EEPROM */
        if(QuaterFlag == 1) {
            QuaterFlag = 0;
            /* Store Current and Voltage of now */
            //to do
        }

        if((MinuteFlag == 1) && (ackFlag == 0)) {
            MinuteFlag = 0;

            CurrentVoltageAcquire(1);
            SettingCurrentReTransmitTimes();
            //update max current
            if(MeanCurrentValue > TodayMaxCurrent) {
                TodayMaxCurrent = MeanCurrentValue;
            }
            //update mean current
            if(TodayMeanCurrent == 0) {
                TodayMeanCurrent = MeanCurrentValue;
            } else {
                TodayMeanCurrent = TodayMeanCurrent / 2 + MeanCurrentValue / 2;
            }
        }

        /* Feed Dog in Every Loop */
        asm("CLRWDT");
    }
    return 0;
}

/*
Initialize Oscillator
 */
void initOsc() {
    /* Page 68 */
    /* 0 1111 0 10 */
    OSCCON = 0x7A; //16M HZ Internal Osc
    return;
}

/*
initialize USART
 */
void initUSART() {
    TRISC4 = 1; //set pin RB1 to signal input
    TRISC5 = 1; //set pin RB2 to signal input

    /* @page117 */
    RXDTSEL = 0; //select pin RC5 as RX
    TXCKSEL = 0; //select pin RC4 as TX

    /* 0010 0100 @PAGE-294*/
    /* Enable Transmit & High Speed Asynchronize Mode */
    TXSTA = 0x24; //Transmit Statues setting

    /* 1001 0000 */
    /* Serial Port Enable & Enable Receiver Under Asynchronize Mode */
    RCSTA = 0x90; //Receive Statues setting

    /* Set the baud rate to 38400bps */
    //BRG16 = 0;//8 BIT Baud Rate Generate
    SPBRGL = 25;
    SPBRGH = 0; //actually this statement are not used

    /* Interrrupt Relatted Bit Settings */
    RCIE = 1; //REceive Interrupt Enable bit,hence we don't have a transmit interrupt
    PEIE = 1; //PEripheral Interrupt Enable bit
    return;
}

/*
initialize ADC@Pin 17
 */
void initVoltageInPin() {
    /* Pin Configuration: */
    ANSC3 = 1; //Set Pin RC3(Pin 7) as ANALOG INPUT
    TRISC3 = 1; //Set Pin RC3 as INPUT
    //RA0 = 0;//Assign a value for Pin 17 ??

    /* Register Configuration: */
    /* 0 00111 0 1 @page-149*/
    /* Enable ADC on AN7 & Enable ADC*/
    ADCON0 = 0x1D;
    /* 1 001 0 0 00 @page-150*/
    /* Right Justify & Fosc/8 & GND is Vref- & VDD is Vref+ */
    ADCON1 = 0x90;

    __delay_ms(100); //changed for debugging
    return;
}

/*
initialize ADC@Pin 17
 */
void initCurrentInPin() {
    /* Pin Configuration: */
    ANSC1 = 1; //Set Pin RC1(Pin 9) as Analog Input
    TRISC1 = 1; //SET PIN RC1 as Input
    //RA0 = 0;//Assign a value for Pin 17 ??

    /* Register Configuration: */
    /* 0 00101 0 1 @page-149*/
    ADCON0 = 0x15; //Enable AN5
    /* 1001 0000 @page-150*/
    ADCON1 = 0x90;

    __delay_ms(100); //changed for debugging
    return;
}

/*
 *initialize DAC for Comparator(Positive Input of Comparator)
 */
void initDAC() {
    /* DAC POWER SOURCE SELECT (Default Value)@Page 159 */
    DACPSS1 = 0;
    DACPSS0 = 0;
    /* DAC register(3.3*2/32 =  0.20625V about 20mA) */
    DACR4 = (DACThreashold & 0x10) >> 4;
    DACR3 = (DACThreashold & 0x08) >> 3;
    DACR2 = (DACThreashold & 0x04) >> 2;
    DACR1 = (DACThreashold & 0x02) >> 1;
    DACR0 = (DACThreashold & 0x01);

    /* Disable Output Connect to DACOUT Pin */
    DACOE = 0;
    /* Enable DAC */
    DACEN = 1;

    __delay_ms(100);
    return;
}

void initCamparator() {
    /* Comparator Negative Input Channel Select bits@Page 172 */
    C1NCH1 = 1; //C12IN2- (Pin 8/RC2)
    C1NCH0 = 0; //C12IN2- (Pin 8/RC2)

    /* Comparator Positive Input Channel Select bit(Select DAC Voltage Reference)@Page 172 */
    C1PCH1 = 0;
    C1PCH0 = 1;

    /* Comparator Speed/Power Select(default:normal(high speed)) */
    C1SP = 0;

    /* Enable Comparator Hysteresis */
    C1HYS = 1;

    /* Comparator Output Enable Bit(Internal Only/Disable Output) */
    C1OE = 0;

    /* Comparator Interrupt on Negative and Positive going Edge Enable Bits */
    C1INTN = 1;
    //C1INTP = 1; //for some Weird Situation

    /* Enable Comparator 1 */
    C1ON = 1;
    /* Enable Comparator Interrupt */
    C1IE = 1;
    /* Clear Flag */
    C1IF = 0;
    return;
}

/*
 **Initialize Timer 0
 **Period : (4*1/16*10^-6)*256*(255 - 0) = 0.01632 second
 */
void initTimer0() {
    /* OPTION_REG @page 176 */
    PSA = 0; //Pre-scaler Assignment bit
    PS2 = 1;
    PS1 = 1;
    PS0 = 1; //Pre-scale is 1:256

    /* Timer 0 Clock Source Select(Internal Instruction Cycle Clock Fosc/4) */
    TMR0CS = 0;

    TMR0 = 0;

    /* Disable Timer 0 Interrupt First */
    T0IE = 0;
    return;
}

/*
 **initialize Timer 1
 **Period:(4/16)*10^(-6)*4*50000 = 50 ms
 */
void initTimer1() {
    /* Set Timer1 Clock Source - Instruction Clock(Fosc/4) @page147*/
    TMR1CS1 = 0;
    TMR1CS0 = 0;

    /* Set Timer1 Clock PreScale Select bits( 1:4 ) */
    T1CKPS1 = 1;
    T1CKPS0 = 0;

    /* Clear Timer1 Overflow Interrupt */
    TMR1IF = 0; //clear flag
    /* Config TMR1H and TMR1L - Counting Register*/
    TMR1H = 0x3C;
    TMR1L = 0xB0;
    /* Disable Timer1 Overflow Interrupt First */
    TMR1IE = 0;

    /* Open Timer1 */
    TMR1ON = 1; //@page 186
    TMR1GE = 0;
    return;
}

/*
Initialize Timer 2
period : (1/4*16*10^-6)*64*16*(200 - 0) = 0.0512 second
 */
void initTimer2() {
    /* @page 191 */
    /* Set Timer2 Clock Pre-Scale Select bits(1:64) */
    T2CKPS1 = 1;
    T2CKPS0 = 1;

    /* Set Timer2 Output Post-Scaler Select bits (1:16)*/
    T2OUTPS3 = 1;
    T2OUTPS2 = 1;
    T2OUTPS1 = 1;
    T2OUTPS0 = 1;

    TMR2ON = 1;
    TMR2IE = 0; //Disable Timer1 overflow interrupt first

    /* Register relate to timing period */
    TMR2 = 0;
    PR2 = 200;
    return;
}

/*
initialize Timer 4
period : (1/4*16*10^-6)*64*16*(200 - 0) = 0.0512 second
 */
void initTimer4() {
    /* @page 191 */
    /* Set Timer4 Clock Pre-Scale Select bits(1:64) */
    T4CKPS1 = 1;
    T4CKPS0 = 1;

    /* Set Timer4 Output Post-Scaler Select bits (1:16)*/
    T4OUTPS3 = 1;
    T4OUTPS2 = 1;
    T4OUTPS1 = 1;
    T4OUTPS0 = 1;

    TMR4ON = 1;
    TMR4IE = 1; //Here we Enable Timer4 Interrupt,make it run permenately

    /* Register relate to timing period */
    TMR4 = 0;
    PR4 = 200;
    return;
}

/*
initialize Timer 6
period : (1/4*16*10^-6)*64*16*(195 - 0) = 0.04992 second
 */
void initTimer6() {
    /* @page 191 */
    /* Set Timer4 Clock Pre-Scale Select bits(1:64) */
    T6CKPS1 = 1;
    T6CKPS0 = 1;

    /* Set Timer4 Output Post-Scaler Select bits (1:16)*/
    T6OUTPS3 = 1;
    T6OUTPS2 = 1;
    T6OUTPS1 = 1;
    T6OUTPS0 = 1;

    TMR6ON = 1;
    TMR6IE = 1; //Here we Enable Timer6 Interrupt,Make it Run With the System

    /* Register relate to timing period */
    TMR6 = 0;
    PR6 = 195;
    return;
}

/*
initialize Watch Time Dog
 */
void initWTD() {
    /* @page 101 */
    WDTCON = 0x1B; //4 seconds
}

unsigned int ADC(unsigned char PinSelect) {
    /* PinSelect select which port to be acquired*/
    unsigned int adval;

    if (PinSelect == 1) CHS1 = 0; //Enable AN5(Pin 9 - CurrentIn Pin)CHS1 = 0;
    else CHS1 = 1; //Enable AN7(Pin 7 - VoltageIn Pin);CHS1 = 1;

    __delay_us(350); //delay time can be changed
    /* start AD conversion */
    ADGO = 1;

    while (ADGO); //judge weather the ADC process finished or not
    adval = ADRESH; //00000000 00000011/* ADC result high bit */
    adval = adval << 8 | ADRESL; //00000011 11111111/* ADC result low bit */

    return adval;
}

/*
Find Maximum Value of a AC voltage signal
 */
void FindMaxValue() {
    unsigned int VoltageValue[AcquiredSampleNum];

    unsigned char k, j, tempIndex = 0;
    unsigned int CurrentValuetemp;
    unsigned int VoltageValuetemp;

    /* Acquire Certian Number of Samples(Current and Voltage) */
    for (k = 0; k < AcquiredSampleNum; ++k) {
        /* if comparator interrupt happen , we just drop previous data and re-acquire */
        if(CurrentAbnormalFlag == 1){
            k = 0;
            CurrentAbnormalFlag = 0;
            #ifdef DEBUG   
                TXREG = 0x99;
                while(!TRMT);
            #endif
        }
            
        CurrentValue[k] = ADC(1);
        VoltageValue[k] = ADC(0);
    }

    /* "Partial" Bubble Sorting */
    /* Current Data Processing */
    for (j = 0; j < 10; ++j) {
        for (k = 0; k < (AcquiredSampleNum - j); k++) {
            if (CurrentValue[k] > CurrentValuetemp) {
                CurrentValuetemp = CurrentValue[k];
                tempIndex = k;
            }
        }
        CurrentValue[tempIndex] = CurrentValue[AcquiredSampleNum - j - 1];
        CurrentValue[AcquiredSampleNum - j - 1] = CurrentValuetemp;
        CurrentValuetemp = 0;
        if( 1 == CurrentAbnormalFlag ) return;
    }
    /* Process to Ignore Abnormal Large Value */
    for (j = 1; j < 8; ++j) {
        if (CurrentValue[AcquiredSampleNum - 1 - j] - CurrentValue[AcquiredSampleNum - 2 - j] > 15)continue; //0.1V ~= 8mA
        else if (CurrentValue[AcquiredSampleNum - j] - CurrentValue[AcquiredSampleNum - 1 - j] > 15) continue;
        else break;

        if( 1 == CurrentAbnormalFlag ) return;
    }
    CurrentPeakValue = CurrentValue[AcquiredSampleNum - 1 - j];//srore results to global variable

    /* Voltage Data Processing */
    tempIndex = 0;
    for (j = 0; j < 5; ++j) {
        for (k = 0; k < (AcquiredSampleNum - j); k++) {
            if (VoltageValue[k] > VoltageValuetemp) {
                VoltageValuetemp = VoltageValue[k];
                tempIndex = k;
            }
        }
        VoltageValue[tempIndex] = VoltageValue[AcquiredSampleNum - j - 1];
        VoltageValue[AcquiredSampleNum - j - 1] = VoltageValuetemp;
        VoltageValuetemp = 0;
        if( 1 == CurrentAbnormalFlag ) return;
    }

    /* There is no need to process voltage data */
    VoltagePeakValue = VoltageValue[AcquiredSampleNum - 1];//store results to global variable

    return;
}

/*
Acquire Current Leakage and Voltage data
*/
void CurrentVoltageAcquire(unsigned char CycleNum) {
    volatile unsigned int maxCurrent;
    //volatile unsigned int minVoltage;
    volatile unsigned char i;
    volatile unsigned char addition = 0;

    //make sure array index smaller than array size
    #assert CycleNum <= SmoothPeriod
    //before we start acquire data , store last time current vlaue
    LastTimeCurrent = MeanCurrentValue;

    FindMaxValue();
    /* Acquire Current Value */
    MeanCurrentValue = CurrentPeakValue;
    maxCurrent = MeanCurrentValue;

    MeanVoltageValue = VoltagePeakValue;
    //minVoltage = MeanVoltageValue;
    VoltageSmoothArray[0] = MeanVoltageValue;

    if (CurrentAbnormalFlag == 1) {
        JumpAway();
        return;
    }

    for(i = 0;i < CycleNum - 1;++i) {
        FindMaxValue();
        /* Acquire Current Value */
        MeanCurrentValue = CurrentPeakValue;

        //as for current , we choose the maximum current leakage value
        if(MeanCurrentValue > maxCurrent) {
            if (((MeanCurrentValue - maxCurrent) > 10) && (i > 0)) {//about 2.5 mA
                i = 0;
                #ifdef DEBUG
                TXREG = 0x88;
                while (!TRMT);
                #endif
            }//about 3mA current difference
            maxCurrent = MeanCurrentValue;//store the maximum current
        }

        /* Acquire Voltage Value */
        MeanVoltageValue = VoltagePeakValue;
        // as for voltage , we store midlle value
        VoltageSmoothArray[i + 1] = MeanVoltageValue;
        //if(MeanVoltageValue < minVoltage)minVoltage = MeanVoltageValue;//store the minumum voltage

        if(CurrentAbnormalFlag == 1) {
            JumpAway();
            return;
        }

    }

    /* Convert to Real Current Value */
    MeanCurrentValue = maxCurrent;
    float_temp = CURRENT_K * MeanCurrentValue + CURRENT_B;//
    MeanCurrentValue = float_temp / 10;
    MeanCurrentValue *= 100;

    //Update maxCurrent and meanCurrent
    if(MeanCurrentValue > TodayMaxCurrent) {
        TodayMaxCurrent = MeanCurrentValue;
    }

    if(CurrentAbnormalFlag == 1) {
        JumpAway();
        return;
    }

    /* Convert to Real Voltage Value */
    //MeanVoltageValue = minVoltage;
    MeanVoltageValue = getMiddleValue(CycleNum);
    if(resetFlag == 0) {//if nRI is 0(a reset instruction has been execuated,ignore fitting voltage)
        addition = 0;
    }
    else if(SystemTime < 2) {
         addition = 54;
    }
    else if(SystemTime < 3) {
        addition = 41;
    }
    else if(SystemTime < 7) {
        addition = 30;
    }
    else if(SystemTime < 13) {
        addition = 19;
    }
    else if(SystemTime < 20) {
        addition = 9;
    }
    else if(SystemTime < 26) {
        addition = 4;
    }

    float_temp = MeanVoltageValue + addition;

    // 80 is about 60V AC Voltage
    if(float_temp > 80) {
        /* Fitting Parameter for Node A */
        float_temp = float_temp * VOLT_K + VOLT_B;//round to integer(add 5)
        MeanVoltageValue = float_temp / 10;//without this division , weird float point calculate result will occur
        MeanVoltageValue *= 100;
    }
    else MeanVoltageValue = 0;

    //Update maxVoltage and minVoltage
    if(TodayMaxVoltage < MeanVoltageValue) {
        TodayMaxVoltage = MeanVoltageValue;
    }
    if(TodayMinVoltage > MeanVoltageValue){
        TodayMinVoltage = MeanVoltageValue;
    }

    return;
}
/*
* get middle value of n period voltage period
*/
unsigned int getMiddleValue(unsigned char len) {
    /* here we use bubble sort , because it's simple */
    unsigned char i = 0,j = 0;

    for (i = 0; i < len - 1; ++i)
    {
        for (j = i + 1; j < len; ++j)
        {
            if(VoltageSmoothArray[i] < VoltageSmoothArray[j]) {
                //swap two value
                VoltageSmoothArray[i] = VoltageSmoothArray[i] ^ VoltageSmoothArray[j];
                VoltageSmoothArray[j] = VoltageSmoothArray[i] ^ VoltageSmoothArray[j];
                VoltageSmoothArray[i] = VoltageSmoothArray[i] ^ VoltageSmoothArray[j];
            }
        }
    }

    if(len % 2 == 0) {
        return (VoltageSmoothArray[(len - 1) / 2] + VoltageSmoothArray[len / 2] ) / 2;
    } else  {
        return VoltageSmoothArray[len / 2];
    }
}
/*
*Transmit data through serial port
*/
void TransmitData(unsigned char Type) {
    int i;
    a[0] = MeanCurrentValue / 256;
    a[1] = MeanCurrentValue % 256;

    b[0] = MeanVoltageValue / 256;
    b[1] = MeanVoltageValue % 256;

    TXREG = startByte;
    while (!TRMT);
    /* Package Length */
    TXREG = sizeof(addr) + 7;
    while (!TRMT);
    for(i = 0;i < sizeof(addr);++i){
        TXREG = addr[i];
        while (!TRMT);
    }
    TXREG = DeviceID;//Device ID
    while (!TRMT);
    TXREG = a[0]; //MSB of Current Value
    while (!TRMT);
    TXREG = a[1]; //LSB of Current Value
    while (!TRMT);
    TXREG = b[0]; //MSB of Voltage Value
    while (!TRMT);
    TXREG = b[1]; //LSB of Voltage Value
    while (!TRMT);
    TXREG = Type;//Type Byte , Indicate the data is what kind of data
    while (!TRMT);
    TXREG = endByte;
    while (!TRMT);

    T2countnum = 0;
    TMR2IE = 1; //Start to counting time , wait for ACK
    waitFlag = 0; //waitting for next waitFlag or ACK Finished

    ackFlag = 1; //re-Set the ackFlag,this means we are waiting for acknowledgement
    return;
}
/*
** Reply Coordinator with required data
** We only transmit once
** buf size is 2
*/
void TransmitParameterData(unsigned char command,unsigned char *buf,unsigned char len){
    int i;
    TXREG = startByte;
    while (!TRMT);
    /* Package Length */
    TXREG = 0x08 + len;
    while (!TRMT);
    for(i = 0;i < sizeof(addr);++i) {
        TXREG = addr[i];
        while (!TRMT);
    } 
    TXREG = command;//MSB of Current Value
    while (!TRMT);
    for(i = 0;i < len;++i) {
        TXREG = *(buf + i);//LSB of Current Value
        while (!TRMT);
    }
    TXREG = endByte;
    while (!TRMT);
}
/*
** Process Received Data  Can we call this function out of interrupt routing ??????
 */
void ReceivedDataProcess() {
    /*
    #ifdef TestUSART
        //reply with received valid data
        int i;
        for(i = 0; i < recNum;++i){
            TXREG = recBuffer[i];
            while (!TRMT);
        }
    #endif
    */

    /* If we received acknowledgement data(ACK Data Length : 11) */
    /* 0x0B addr(6 byte) DeviceID Command_Type Current_MSB Current_LSB */
    if (recNum == 11 && (isAddressMatch(recBuffer,1) == 0 || DeviceID == recBuffer[sizeof(addr) + 1]) && (recBuffer[sizeof(addr) + 2] == 0x04) && (recBuffer[sizeof(addr) + 3] == a[0]) && (recBuffer[sizeof(addr) + 4] == a[1])) {
        ackFlag = 0;// ackFlag
        #ifdef TestUSART
            TXREG = 0x55;
            while (!TRMT);
            TXREG = 0x55;
            while (!TRMT);
        #endif
        /* added @ 06_09 */
        T2countnum = 0;
        TMR2IE = 0;

        waitFlag = 0; //added in this new Version Code
        reTransmitTimes = 0; //clear re-transmit times when ACK completed
        return;
    }

    /* realtime data command*/
    /* 0x0A address(6 byte) DeviceID 0xCC 0x00 - (the command byte is 0xCC now) */
    if ((recNum == 10) && (recBuffer[sizeof(addr) + 2] == 0xCC) ) {
        /* Address match or id match */
        if (isAddressMatch(recBuffer,1) == 0 || DeviceID == recBuffer[sizeof(addr) + 1]) {
            reTransmitTimes = 0; //clear re-transmit times when we need to transmit real-time data to Coordinator
            #ifdef DEBUG
                TXREG = 0x39;
                while (!TRMT);
            #endif
            //if there is a Comparator Caused Acquire Action 1 Second Ago , We Just Tack that Result
            if(CurrentContinueAbnormalFlag == 0) {
            #ifdef DEBUG
                TXREG = 0x77;
                while (!TRMT);
            #endif
                CurrentVoltageAcquire(SmoothPeriod);//if Current is abnormal , we don't need to acquire data
                //SettingCurrentReTransmitTimes();
                //MeasurementCurrentPeriod = 6000;//about 5 minutes , to avoid sending data during real-time acquiring
            }
            /* real-time echo */
            DataFrameType = 0x02;
            TransmitData(DataFrameType);
        }
        MeasurementCurrentPeriod = 5000;//about 5 minutes , to avoid sending data during real-time acquiring
        /* Clear Count Number */
        T1countnum = 0;
        T0countnum = 0;
        /* Enable Timer1 Overflow Interrupt */
        TMR1IE = 1;
        T0IE = 1;
        /* Forbidden for Acquire Current Data */
        ForbiddenSendCurrentFlag = 1;
        ForbiddenSendVoltageFlag = 1;
        return;
    }

    /* new Day handler(new day comming 23:59:59 -> 00:00:00) */
    /* 0x05 0xFF 0xFF 0xFF 0xFF */
    if((recNum == 5) && (BroadcastByte == recBuffer[1]) && (BroadcastByte == recBuffer[2]) && (BroadcastByte == recBuffer[3]) && (BroadcastByte == recBuffer[4])) {
        //reset data
        TodayMaxCurrent = 0;
        TodayMeanCurrent = 0;
        TodayCurrentExceedTime = 0;
        TodayPowerDownTime = 0;
        TodayMinVoltage = 24000;
        TodayMaxVoltage = 0;
        asm("RESET");//only reset when there are not data wait to be acknoledgement
        return;
    }

    /* Parameter Setting Command */
    /* 10 address(6 byte) DeviceID command-byte command-param */
    if((recNum == 10) && (isAddressMatch(recBuffer,1) == 0 || DeviceID == recBuffer[sizeof(addr) + 1])) {
        //Setting current threashold
        if(recBuffer[sizeof(addr) + 2] == 0x01) {
            //read param command
            if(recBuffer[sizeof(addr) + 3] == 0x00){
                TransmitParameterBuf[0] = CurrentThreshold / 100;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,1);
            }
            //modify param command
            else CurrentThreshold = recBuffer[sizeof(addr) + 3] * 100;
        //Setting voltage upper range
        } else if(recBuffer[sizeof(addr) + 2] == 0x02) {
            //read param command
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = VoltageUpperRange/ 100;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,1);
            }
            //modify param command
            else {
                VoltageUpperRange = recBuffer[sizeof(addr) + 3] * 100;
            }
        //Setting voltage down range
        } else if(recBuffer[sizeof(addr) + 2] == 0x03) {
            //read param command
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = VoltageDownRange / 100;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,1);
            }
            //modify param command
            else {
                VoltageDownRange = recBuffer[sizeof(addr) + 3] * 100;
            }
        //Read Current Abnormal Time(Count) -- Read Only
        } else if(recBuffer[sizeof(addr) + 2] == 0x05) {
            //read param command
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayCurrentExceedTime / 256;
                TransmitParameterBuf[1] = TodayCurrentExceedTime % 256;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,2);
            }
        //Read Mean Current Value -- Read Only
        } else if(recBuffer[sizeof(addr) + 2] == 0x06) {
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayMeanCurrent / 100;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,1);
            } 
        //Read Today Maximum Current Value
        } else if(recBuffer[sizeof(addr) + 2] == 0x07){
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayMaxCurrent / 100;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,1);
            } 
        }else if(recBuffer[sizeof(addr) + 2] == 0x08) {
            //read param command
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayMinVoltage / 256;
                TransmitParameterBuf[1] = TodayMinVoltage % 256;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,2);
            } 
        } else if(recBuffer[sizeof(addr) + 2] == 0x09) {
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayMaxVoltage / 256;
                TransmitParameterBuf[1] = TodayMaxVoltage % 256;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,2);
            } 
        } else if(recBuffer[sizeof(addr) + 2] == 10){
            if(recBuffer[sizeof(addr) + 3] == 0x00) {
                TransmitParameterBuf[0] = TodayPowerDownTime / 256;
                TransmitParameterBuf[1] = TodayPowerDownTime % 256;
                TransmitParameterData(recBuffer[sizeof(addr) + 2],TransmitParameterBuf,2);
            } 
        }

        return;
    }
    
    /* Command for echo (debug purpose)*/
    /* 0x03 ID 0xDD */
    if ((3 == recNum) && (DeviceID == recBuffer[1]) && (0xDD == recBuffer[2])) {
        TXREG = 0x98;
        while (!TRMT);
        return;
    }

    /* in other case,we just treat recData as garbage data,do nothing */
    return;
}
/*
** Check Received Package Address Field
** return 0 for matched , return 1 for not matched
*/
int isAddressMatch(unsigned char *p,int startIndex){
    int i;
    for(int i = 0; i < sizeof(addr);++i) {
        //Broadcast Address
        if(0xFF == *(p + startIndex + i)) continue;
        //Address not match
        if(addr[i] != *(p + startIndex + i)) return 1;
    }
    return 0;
}
/*
** Changing Current Measurement Period Dynamiclly
*/
void SettingCurrentReTransmitTimes() {
    //(0.05*100 = 5)about 5 second(Forbidden Send Current Period)

    if (MeanCurrentValue > 10000) MeasurementCurrentPeriod = 600;//30 seconds
    else if (MeanCurrentValue > 5000) MeasurementCurrentPeriod = 1200;//1 minutes
    else if (MeanCurrentValue > 3000) MeasurementCurrentPeriod = 2400;//2 minutes
    else if (MeanCurrentValue > 2000) MeasurementCurrentPeriod = 6000;//5 minutes
    else MeasurementCurrentPeriod = 12000;//10 minutes

    /* Changing Retransmit Period */
    //MeasurementCurrentPeriod = ( MeasurementCurrentPeriod * 50 ) / RetransmitTimeRatio;

    return;
}
/*
** Read 1 Byte from on-chip eeprom
*/
unsigned char eeprom_read_byte(unsigned char addr){
    EEADR = addr; //Address to be read
    EECON1bits.EEPGD = 0;//Selecting EEPROM Data Memory
    EECON1bits.RD = 1; //Initialise read cycle

    while(EECON1bits.RD == 1);

    return EEDATA; //Returning data
}
/*
** Write 1 Byte to on-chip eeprom
*/
void eeprom_write_byte(unsigned char addr,unsigned char data){
    unsigned char INTCON_SAVE;//To save INTCON register value

    EEADR = addr; //Address to write
    EEDATA = data; //Data to write

    EECON1bits.EEPGD = 0; //Selecting EEPROM Data Memory
    EECON1bits.WREN = 1; //Enable writing of EEPROM
    INTCON_SAVE = INTCON;//Backup INCON interupt register

    /* Notice !! here Disable interrupt , so we can't afford frequently write eeprom */

    INTCON = 0; //Diables the interrupt
    EECON2 = 0x55; //Required sequence for write to internal EEPROM
    EECON2 = 0xAA; //Required sequence for write to internal EEPROM
    EECON1bits.WR = 1; //Initialise write cycle

    INTCON = INTCON_SAVE;//Enables Interrupt
    EECON1bits.WREN = 0; //To disable write
    while(PIR2bits.EEIF == 0) {//Checking for complition of write operation
      asm("nop"); //do nothing
    }

    PIR2bits.EEIF = 0; //Clearing EEIF bit
}
/*
** Interrupt Routing
*/
static void interrupt ISR() {
    unsigned char temp;
    /* Transmit Received Data */
    if (RCIE && RCIF) {
        /* First,store received 1 byte data */
        temp = RCREG;
        /* Overrun Error */
        if (OERR == 1) {
            temp = RCREG;
            temp = RCREG;
            CREN = 0;
            CREN = 1;
            #ifdef DEBUG
                TXREG = 0x91;
                while(!TRMT);
            #endif
        }

        /* Valid Data ? Index Overflow ? */
        if ((startFlag == 1) && (index < recBufferSize - 1)) {
            recBuffer[index] = temp;//store data
            if(index == 0) {
                if(temp > 25)startFlag = 0;
                else recNum = temp;
            }
            else if (index >= recNum) {
                if(temp == endByte) {
                    //received valid data
                    recFlag = 1;
                }
                index = 0;
                startFlag = 0;
            }
            ++index;
        }

        /* Here We Decide Weather Received Data are Valid */
        if (temp == startByte && startFlag == 0) {
            startFlag = 1; //when we received a start byte,set startFlag
            index = 0; //initialize index,very important
        }
    }

    /* Comparator */
    if (C1IE && C1IF) {
        /* Clear Flag */
        C1IF = 0;
        /* Set Receive Flag */
        CurrentAbnormalFlag = 1;
        /* Disable Interrupt */
        C1IE = 0;
    }

    /* Fairly Low Priority Interrupt */
    /* Timer0 Interrupt Routing */
    if ( T0IE && T0IF )//If The Interrupt is about Timer0
    {
        T0IF = 0; //clear flag
        T0countnum++; //counting number
        if (T0countnum >= Timer0ExpireNum) {
            T0countnum = 0;
            T0IE = 0; //Disable Timer0 Interrupt
            //TO DO
            ForbiddenSendVoltageFlag = 0; //release forbidden
        }
    }

    /* Timer1 Interrupt Routing */
    if (TMR1IE && TMR1IF)//If The interrupt  is about Timer1
    {
        /* Clear Timer1 Interrupt Flag */
        TMR1IF = 0;
        /* Increase Count Variable */
        T1countnum++;
        /* Re-Load Counting Data */
        TMR1H = 0x3C;
        TMR1L = 0xB0;

        if (T1countnum >= MeasurementCurrentPeriod) {
            T1countnum = 0;
            /* Disable Timer1 Overflow Interrupt  */
            TMR1IE = 0;
            /* release forbidden for acquire current data */
            ForbiddenSendCurrentFlag = 0;
        }
    }

    /* Timer2 Interrupt Routing */
    if (TMR2IE && TMR2IF)//if the interrupt  is about Timer2
    {
        TMR2IF = 0; //Clear Flag
        T2countnum++; //Counting Number
        if (T2countnum >= Timer2ExpireTime) {
            T2countnum = 0;
            TMR2IE = 0; //Disable Timer2 Interrupt
            waitFlag = 1; //Set WaitFlag
        }
    }

    /* Timer4 Interrupt Routing(Measuring CurrentInPin and VoltageInPin Periodicaly) */
    if (TMR4IE && TMR4IF)//if the interrupt  is about Timer2
    {
        TMR4IF = 0; //Clear Flag
        ++T4countnum; //Counting Number
        if (T4countnum >= Timer4ExpireNum) {
            T4countnum = 0;
            TMR4IE = 0; //Disable Timer2 Interrupt
            //TXREG = 0x33;
            //while (!TRMT);
            TimeToAcauireFlag = 1;//there is no need to repeat Acquiring
        }
    }

    /* Timer6 Interrupt Routing */
    if (TMR6IE && TMR6IF)//if the interrupt  is about Timer2
    {
        TMR6IF = 0; //Clear Flag
        T6countnum++; //Counting Number
        if (T6countnum >= Timer6ExpireNum) {
            //Clear Counter
            T6countnum = 0;
            MinuteFlag = 1;
            //System time in minute
            if(SystemTime < 1440) ++SystemTime;//about 24 hour for test
            else {
                if(ackFlag == 0) {
                    asm("RESET");//only reset when there are not data wait to be acknoledgement
                    //after reset SystemTime will reset to 0
                }
            }
            /* certain Acquire will send voltage Monitor value half hour */
            if( (DeviceID == MonitorVoltageID) && (0 == SystemTime % 30) ) {
                /* Set Flag bit every half hour */
                MonitorVoltageFlag = 1;
            }
            /* 15 minutes comming now */
            if(0 == SystemTime % 15) {
                QuaterFlag = 1;
            }

            //TXREG = SystemTime%256;
            //while (!TRMT);
        }
        //Feed Dog Every 3 Second
        WatchTimerDogCountnum++;
        if ( WatchTimerDogCountnum >= 60 ) {
            WatchTimerDogCountnum = 0;
            //asm("CLRWDT");
        }
        //Flag that Indicate CPU Interrupted by "Comparator Interrpt" Frequently !
        if( 1 == CurrentContinueAbnormalFlag ) {
            /* count only need to count */
            ++CurrentContinueAbnormalCount;
            if(CurrentContinueAbnormalCount > 20) {//about 1 second
                CurrentContinueAbnormalCount = 0;
                CurrentContinueAbnormalFlag = 0;
            }
        }
    }
}
