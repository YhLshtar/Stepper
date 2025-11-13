#ifndef __MT6701_H
#define __MT6701_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#define 	USE_I2C_READ_ANGLE				1

#define		ENCODER_MAX_VALUE					16384																	//编码器最大值
#define		ENCODER_MAX_ERROR				 (ENCODER_MAX_VALUE * 0.6f / 360.0f)		//编码器最大误差 : 0.75°
#define   CALIBRATION_RANGE					200
#define   ANGLE_DIFF_THEO						82			//编码器校准时，相邻角的理论差值 (ENCODER_MAX_VALUE / CALIBRATION_RANGE)
#define		RT_ANGLE_ERROR_MAX			 (ANGLE_DIFF_THEO * 0.25)								//理论和实际角的误差最大值

enum ANGLE_MODE
{
	USE_RELATIVE_ANGLE,		//相对角度
	USE_ABSOLUTE_ANGLE		//绝对角度
};

typedef void (*motor_run)(unsigned char axis, unsigned char dir,unsigned long m_steps);

bool	MT6701_Init(char angle_mode);
bool	isCaliDataExist(void);
bool  MT6701_INL_Calibration(unsigned char axis, unsigned char dir_z, unsigned char dir_f,
														 unsigned long m_steps, motor_run function);//axis是为多个MT6701预留参数
bool  MT6701_MDA_Calibration(unsigned char axis);
int   Calibration_Angle_Read(unsigned char axis);
float Machinery_Angle(int angle);

#ifdef __cplusplus
}
#endif

#endif /* __MT6701_H */