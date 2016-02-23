/*
 * File:   parameter.h
 * Author: dan
 *
 * Created on 2015?11?27?, ??4:13
 */

#ifndef PARAMETER_H
#define	PARAMETER_H

#define DeviceID 9 //Device ID of this Router

#ifdef	__cplusplus
extern "C" {
#endif

#if 1 == DeviceID
	#define CURRENT_K 4.2995
	#define CURRENT_B -20.7
	#define VOLT_K 4.2463
	#define VOLT_B 158.8
#endif

#if 2 == DeviceID
	#define CURRENT_K 4.3709
	#define CURRENT_B 2.6
	#define VOLT_K 4.2735
	#define VOLT_B 116.1
#endif

#if 3 == DeviceID
	#define CURRENT_K 4.4287
	#define CURRENT_B -19.6
	#define VOLT_K 4.0486
	#define VOLT_B 162.5
#endif

#if 4 == DeviceID
	#define CURRENT_K 4.2835
	#define CURRENT_B -13.8
	#define VOLT_K 3.9604
	#define VOLT_B 215.3
#endif

#if 5 == DeviceID
	#define CURRENT_K 4.2900
	#define CURRENT_B -16.3
	#define VOLT_K 3.9878
	#define VOLT_B 196.7
#endif

#if 6 == DeviceID
	#define CURRENT_K 4.0489
	#define CURRENT_B -16.3
	#define VOLT_K 3.8573
	#define VOLT_B 225.8
#endif
/* 9? */
#if 7 == DeviceID
	#define CURRENT_K 4.2900
	#define CURRENT_B -29.2
	#define VOLT_K 4.0691
	#define VOLT_B 173.7
	volatile unsigned char addr[] = {0x10,0x28,0x67,0x21,0x91,0x05};
#endif
/* 10? */
#if 8 == DeviceID
	#define CURRENT_K 4.2303
	#define CURRENT_B -17.9
	#define VOLT_K 4.1081
	#define VOLT_B 188.3
	volatile unsigned char addr[] = {0x86,0x28,0x48,0x45,0x82,0x72};
#endif

/* 13? */
#if 9 == DeviceID
	#define CURRENT_K 4.1773
	#define CURRENT_B -5.0
	#define VOLT_K 3.9800
	#define VOLT_B 198.7
	volatile unsigned char addr[] = {0x54,0x37,0x10,0x66,0x03,0x52};
#endif
/* 14? */
#if 10 == DeviceID
	#define CURRENT_K 4.1034
	#define CURRENT_B -27.1
	#define VOLT_K 3.9879
	#define VOLT_B 211.2
	volatile unsigned char addr[] = {0x59,0x85,0x64,0x01,0x49,0x74};
#endif
/* 15? */
#if 11 == DeviceID
	#define CURRENT_K 4.1713
	#define CURRENT_B -10.9
	#define VOLT_K 4.1152
	#define VOLT_B 179.0
	volatile unsigned char addr[] = {0x39,0x62,0x64,0x73,0x47,0x38};
#endif

#if 12 == DeviceID
	#define CURRENT_K 4.3454
	#define CURRENT_B -20.9
	#define VOLT_K 4.1665
	#define VOLT_B 197.3
#endif

#if 13 == DeviceID
	#define CURRENT_K 4.0489
	#define CURRENT_B -8.2
	#define VOLT_K 4.2780
	#define VOLT_B 100.2
#endif

#if 14 == DeviceID
	#define CURRENT_K 4.4866
	#define CURRENT_B -19.8
	#define VOLT_K 3.9017
	#define VOLT_B 234.2
#endif

#if 15 == DeviceID
	#define CURRENT_K 4.2900
	#define CURRENT_B -24.9
	#define VOLT_K 4.0403
	#define VOLT_B 177.2
#endif

#if 16 == DeviceID
	#define CURRENT_K 4.2303
	#define CURRENT_B -13.6
	#define VOLT_K 4.1494
	#define VOLT_B 204.7
#endif

#if 17 == DeviceID
	#define CURRENT_K 4.2303
	#define CURRENT_B -5.2
	#define VOLT_K 4.1841
	#define VOLT_B 101.1
#endif

#if 18 == DeviceID
	#define CURRENT_K 4.0153
	#define CURRENT_B -8.0
	#define VOLT_K 3.8162
	#define VOLT_B 258.6
#endif

#if 19 == DeviceID
	#define CURRENT_K 4.0695
	#define CURRENT_B 5.7
	#define VOLT_K 4.2091
	#define VOLT_B 95.4
#endif

#if 20 == DeviceID
	#define CURRENT_K 4.1204
	#define CURRENT_B -6.6
	#define VOLT_K 3.9603
	#define VOLT_B 248.9
#endif

#if 21 == DeviceID
	#define CURRENT_K 4.1204
	#define CURRENT_B -14.9
	#define VOLT_K 4.3954
	#define VOLT_B 159.1
#endif

#ifdef	__cplusplus
}
#endif

#endif
