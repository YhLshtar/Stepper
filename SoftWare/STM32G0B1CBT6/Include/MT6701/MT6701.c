#include "MT6701.h"
#if USE_I2C_READ_ANGLE
#include "i2c.h"
#else
#include "gpio.h"
#endif
#include "m_flash.h"
#include "Common.h"

struct  ANGLE_CONFIG
{
	char  mode;
	int   zero_angle;
	float zero_angle_mach;
	volatile int current_angle;
  volatile int angle;
	volatile int motor_calib_angle;
}Angle;

HAL_StatusTypeDef I2CRecvStatus;

volatile short	angle_r 			=  0;
unsigned char		angle_data[2] = {0};
bool						isCalibration =  false;
int							calib_point[CALIBRATION_RANGE + 1 + 1];

static bool MT6701_GetAngleData(int* value);
static int  MT6701_ReadAngle(unsigned char axis);

bool MT6701_Init(char angle_mode)
{
	HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_SET);
	
	Angle.mode = angle_mode;
	
	if(!MT6701_GetAngleData(&Angle.zero_angle))
	{
		return false;
	}
	Angle.zero_angle_mach = Machinery_Angle(Angle.zero_angle);
	Angle.current_angle = Angle.zero_angle;
	if(Angle.current_angle == 0)
	{
		return false;
	}
	if(Angle.mode == USE_RELATIVE_ANGLE)		//相对角度
	{
		Angle.angle = Angle.current_angle - Angle.zero_angle;
	}
	else																		//绝对角度
	{
		Angle.angle = Angle.current_angle;
	}
	//没下电就复位的情况下，DAM I2C值一直读不到，直接废了。但是直接断电复位情况下没出现过读不到
	I2CRecvStatus = HAL_I2C_Mem_Read_DMA(&hi2c1, (0x06 << 1 | 0), 0x03, I2C_MEMADD_SIZE_8BIT, 
																			 &angle_data[0], sizeof(angle_data)/sizeof(angle_data[0]));
	if(I2CRecvStatus != HAL_OK)
	{
		return false;
	}
	
	return true;
}
//最大支持IIC和SSI接口14位ADC
bool MT6701_GetAngleData(int* value)
{
#if USE_I2C_READ_ANGLE
	I2CRecvStatus = HAL_I2C_Mem_Read(&hi2c1, (0x06 << 1), 0x03, I2C_MEMADD_SIZE_8BIT, //不用关心读写位，函数自动完成
																	 &angle_data[0], 2, 100);
	if(I2CRecvStatus != HAL_OK)
	{
		return false;
	}
#else
	HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_SET);
#endif
	angle_r = angle_data[0] << 6 | angle_data[1] >> 2;
	// *value = (angle_r / 16384.0f) * 360.0f ;
	*value = angle_r;
	
	return true;
}

int MT6701_ReadAngle(unsigned char axis)
{
	if(Angle.mode == USE_RELATIVE_ANGLE)		//相对角度
	{
		Angle.angle = (Angle.zero_angle - Angle.current_angle + ENCODER_MAX_VALUE) % ENCODER_MAX_VALUE;
	}
	else																		//绝对角度
	{
		Angle.angle = Angle.current_angle;
	}

	return Angle.angle;
};

float Machinery_Angle(int angle)
{
	return (float)angle / 16384.0f * 360.0f;
}

static int angle_diff(int x, int y)
{
	int abs = x - y;
	abs = (abs > 0) ? abs : -abs;
  if (abs > ENCODER_MAX_VALUE / 2)		//特殊情况分别在0°前后，如0.1 和 359.8
	{
  	abs = ENCODER_MAX_VALUE - abs;
	}

	return abs;
}

static int angle_average(int x, int y)
{
	int average = (x + y) / 2;
	int abs = x - y;
	abs = (abs > 0) ? abs : -abs;
	if (abs > ENCODER_MAX_VALUE / 2)
	{
		average = average + ENCODER_MAX_VALUE / 2;	//调整平均方向
	}
	if (average > ENCODER_MAX_VALUE)
	{
		average = average - ENCODER_MAX_VALUE;
	}

	return average;
}

bool isCaliDataExist(void)
{
	bool ret = false;
	ret = Read_Specified_Flash(CALIBRATION_DATA, calib_point,
										sizeof(calib_point) / sizeof(calib_point[0]));

	if (!ret)
	{
		return false;
	}

	isCalibration = true;

	return true;
}
//传感器非线性度计算
bool MT6701_INL_Calibration(unsigned char axis, unsigned char dir_z, unsigned char dir_f,
														unsigned long m_steps, motor_run function)
{
	// calib_point[200] = MT6701_ReadAngle(axis);
	/*正转一圈，反转一圈。
	 *数组0对应理论机械角度0°(360°)的编码器采集值
	 *数组199对应理论机械角度358.2°的编码器采集值*/
	for (short i = 0; i < CALIBRATION_RANGE; i++)			//正
	{
		calib_point[i] = MT6701_ReadAngle(axis);
		function(axis, dir_f, m_steps);
		HAL_Delay(50);
	}
	if (calib_point[0] > ANGLE_DIFF_THEO * 1 && calib_point[0] < ANGLE_DIFF_THEO * 1)
	{
		return false;
	}
	short zero_index = 0;
	for (short i = 1; i < CALIBRATION_RANGE; i++)
	{
		if (calib_point[i] < calib_point[i - 1] &&
			  calib_point[i] < calib_point[i + 1])
		{
			zero_index = i;
			break;
		}
	}
	//机械零点找的有问题，偏差太大，大于3.6°;也有可能传感器出问题了(几乎不太可能)
	if (zero_index >= 2)
	{
		return false;
	}
	function(axis, dir_z, m_steps);
	HAL_Delay(1000);
	for (short i = CALIBRATION_RANGE - 1; i >= 0; i--)//反
	{
		int angle = MT6701_ReadAngle(axis);
		if (angle_diff(calib_point[i], angle) >= ENCODER_MAX_ERROR)
		{
			__LOG__("Error: %d %d %d\n", i, calib_point[i], angle);
			return false;
		}
		calib_point[i] = angle_average(calib_point[i], angle);
		if (i == 0) break;
		function(axis, dir_z, m_steps);
		HAL_Delay(50);
	}
	//检查相邻之间差值是否符合理论值
	int current, next;
	for (short i = 0; i < CALIBRATION_RANGE; i++)
	{
		current = calib_point[i];
		if (i == CALIBRATION_RANGE - 1)
		{
  		next = calib_point[0];
		}
		else
		{
     	next = calib_point[i + 1];
		}
		int diff1 = angle_diff(current, next);
		int diff2 = diff1 - ANGLE_DIFF_THEO;//相邻角的实际差值 - 相邻角的理论差值
		diff2 = (diff2 > 0) ? diff2 : -diff2;
		if (diff2 > RT_ANGLE_ERROR_MAX)
		{
			return false;
		}
	}
	// //处理跳变点，使数组从下标0开始递增
	// for (short i = 0; i < CALIBRATION_RANGE - 1; i++)
	// {
	// 	int min_idx = i;
	// 	for (int j = i + 1; j < CALIBRATION_RANGE; j++)
	// 	{
	// 		if (calib_point[j] < calib_point[min_idx])
	// 		{
	// 			min_idx = j;
	// 		}
	// 	}
	// 	// 交换找到的最小元素
	// 	current = calib_point[min_idx];
	// 	calib_point[min_idx] = calib_point[i];
	// 	calib_point[i] = current;
	// }
	//数组200对应理论机械角度360°(0°)的编码器采集值
	if (calib_point[0] > calib_point[1])
	{
		calib_point[200] = calib_point[0];
		calib_point[0]   = calib_point[0] - ENCODER_MAX_VALUE;
	}
	else
	{
		calib_point[200] = calib_point[0] + ENCODER_MAX_VALUE;
	}
	calib_point[201] = calib_point[200] + calib_point[1] - calib_point[0];

	if (!Write_Specified_Flash(CALIBRATION_DATA, calib_point,sizeof(calib_point) / sizeof(calib_point[0])))
	{
		return false;
	}

	isCalibration = true;

	return true;
}
//传感器安装误差导致的磁偏角计算。因为没有标定工具，所以待定 TODO
bool MT6701_MDA_Calibration(unsigned char axis)
{

	return false;
}
//返回经过校准后的角度
int Calibration_Angle_Read(unsigned char axis)
{
	int angle = MT6701_ReadAngle(axis);
	if (isCalibration)
	{
		// 处理跳变区
		if (angle < calib_point[0])
		{
			angle += ENCODER_MAX_VALUE;
		}
		// 二分法查找扇区
		int  Sector = 0;
		int  Left   = 0;
		int  Right  = CALIBRATION_RANGE;
		while (Left <= Right)
		{
			int mid = Left + ((Right - Left) >> 1);
			if (angle >= calib_point[mid])
			{
				Sector = mid;
				Left   = mid + 1;
			}
			else
			{
				Right  = mid - 1;
			}
		}
		// 已经提前处理了 calibration[200] = calibration[0] + ENCODER_MAX;
		Left  = calib_point[Sector];
		Right = calib_point[Sector + 1];//极限情况例如1.8° sector = 1 360° sector = 200
		// 选择(Right - Left) or ANGLE_DIFF_THEO ?
		int delta = 256 * (angle - Left) / ANGLE_DIFF_THEO;
		Left  = ANGLE_DIFF_THEO * Sector - Left;
		Right = ANGLE_DIFF_THEO * (Sector + 1) - Right;
		int Error = Left + ((Right - Left) * delta >> 8);
		// printf("sec %d err1 %d err2 %d err3 %d\n", Sector, Left, Right, Error);
		angle = angle + Error;
	}
	// unsigned int mask = (ENCODER_MAX_VALUE - angle) >> 31;
	// angle -= mask & (unsigned int)ENCODER_MAX_VALUE;
	Angle.motor_calib_angle = angle;

	return Angle.motor_calib_angle;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) 
{
	if (hi2c->Instance == I2C1) 
	{
		angle_r = angle_data[0] << 6 | angle_data[1] >> 2;
		Angle.current_angle = angle_r ;
		I2CRecvStatus = HAL_I2C_Mem_Read_DMA(&hi2c1, (0x06 << 1 | 0), 0x03, I2C_MEMADD_SIZE_8BIT, 
																				 &angle_data[0], sizeof(angle_data)/sizeof(angle_data[0]));
	}
}

