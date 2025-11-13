#ifndef __M_TIMERS_H__
#define __M_TIMERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32g0xx_hal.h"


#define     TIMER1     			1
#define     TIMER2     			2
#define			TIMER3 					3
#define     TIMER4     			4
#define     TIMER5     			5
#define     TIMER6     			6
#define     TIMER7     			7
#define			TIMER8 					8
#define     TIMER9     			9
#define     TIMER10    			10
#define			TIMER11 				11
#define     TIMER12     		12
#define     TIMER13     		13
#define			TIMER14 				14
#define     TIMER15     		15
#define     TIMER16     		16
#define			TIMER17 				17
#define     TIMERDWT        99

#define     TIME_TIMER      3
#define     TIME_DWT        2
#define     TIME_NONE       1

#define			DelayTimer			htim3
#define			isTimerDelay 		TIME_TIMER
#if				 (isTimerDelay == TIME_NONE)
#define			UsBasicCount		72
#define     CPUFrequent			72000000
#endif

void Timers_Start(unsigned char index);
	
void timer_delay_us(unsigned short t);
void timer_delay_ms(unsigned short t);
void timer_delay_s (unsigned short t);

#ifdef __cplusplus
}
#endif
#endif /*__M_TIMERS_H__ */