#ifndef __TMC2209_H
#define __TMC2209_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32g0xx_hal.h"

typedef enum 
{
	MicroStep_Full = 1,
	MicroStep_2 	 = 2,
	MicroStep_4 	 = 4,
	MicroStep_8 	 = 8,
	MicroStep_16 	 = 16,
	MicroStep_32 	 = 32,
	MicroStep_64 	 = 64,
	MicroStep_128  = 128,
	MicroStep_256  = 256
}MicroStepType;

#define 	MotorFWD					1
#define 	MotorREV					0
#define 	MicroStep					MicroStep_16
#define   OneRoundStep			200
//RPM(转/分)
#define   Speed2Freq(rpm)		(unsigned int)((rpm * 360 * MicroStep / 1.8) / 60)
//根据定时器配置，转为重装载需要的值
#define   ToPreiod(r_freq)	(unsigned int)(r_freq <= 0 ? 0 : (64000000 / (32 * r_freq)))	

void TMC2209_StepFreq_OnlyLevel(unsigned char level);
void TMC2209_StepFreq_Set(unsigned int hz);
void TMC2209_MotorDir(unsigned char dir);
void TMC2209_Spread(bool enable);
void TMC2209_Stdby(bool enable);
void TMC2209_En(bool enable);

void TMC2209_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TMC2209_H */