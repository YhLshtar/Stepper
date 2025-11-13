#include "tim.h"
#include "Stepper_control.h"
#include "MT6701.h"
#include "PIDControl.h"
#include "Common.h"

#if LIMIT_HARD_SWITCH_EN
#define  ReadLimitIOL()					HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define  ReadLimitIOR()					HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#endif
#define  Speed_Balance_Point		275										//rpm
#define  DefaultMinFreq   			10   									//默认最低脉冲频率hz.
#define  Pulse				 				 (200 * MicroStep)			//转一圈,脉冲数量.

#define  LockedCountMax					5																//堵转计数阈值
#define	 LockedThreshold 			 (ENCODER_MAX_VALUE * 0.00055)		//编码器9 约等于0.2°
#define  LoseStepCountMax				10															//丢步计数阈值
#define  LoseStepThreshold(x)	 (x / 6000 * ENCODER_MAX_VALUE)		//(rpm / 60) * 360° * 0.01(s)
#define	 DynamicLagAngle			 (ENCODER_MAX_VALUE / 450 - 2)		//编码器34.5 约等于0.76°
/*--------------------------------------------------------------------------------*/
void OS_LoadTimer(unsigned char axis, float m_freq);  //定时器加载频率
void OS_LRamp(unsigned char axis);       							//直线加减速.
void OS_SRamp(unsigned char axis);       							//S曲线加减速.
void OS_NRamp(unsigned char axis);     								//无加减速.
void OS_SetPowerIO(unsigned char axis, unsigned char m_level);
void OS_Direction(unsigned char axis, unsigned char m_dir);
bool HardLSW_OS_Limit(unsigned char axis); 						//硬件限位开关函数.
// bool SoftLSW_OS_Limit(unsigned char axis);   					//软件限位
void PID_Parameter_Init(unsigned char axis);
/*--------------------------------------------------------------------------------*/
struct	STRUCT_RAMP					Ramp[Motor_Number];
struct	STRUCT_SYSTEM   		System[Motor_Number];
struct	STRUCT_CONFIG   		Config[Motor_Number];
struct	PIDParameter_Incr		PIDPara_Incr[Motor_Number];
//S曲线表
const float SRampTable[100] =
{
	0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.02f, 0.02f, 0.02f,
	0.02f, 0.02f, 0.03f, 0.03f, 0.03f, 0.04f, 0.04f, 0.04f, 0.05f, 0.05f,
	0.06f, 0.06f, 0.07f, 0.08f, 0.08f, 0.09f, 0.10f, 0.11f, 0.12f, 0.13f,
	0.14f, 0.15f, 0.17f, 0.18f, 0.20f, 0.21f, 0.23f, 0.25f, 0.27f, 0.29f,
	0.31f, 0.33f, 0.35f, 0.38f, 0.40f, 0.43f, 0.45f, 0.48f, 0.50f, 0.52f,
	0.55f, 0.57f, 0.60f, 0.62f, 0.65f, 0.67f, 0.69f, 0.71f, 0.73f, 0.75f,
	0.77f, 0.79f, 0.80f, 0.82f, 0.83f, 0.85f, 0.86f, 0.87f, 0.88f, 0.89f,
	0.90f, 0.91f, 0.92f, 0.92f, 0.93f, 0.94f, 0.94f, 0.95f, 0.95f, 0.96f,
	0.96f, 0.96f, 0.97f, 0.97f, 0.97f, 0.98f, 0.98f, 0.98f, 0.98f, 0.98f,
	0.99f, 0.99f, 0.99f, 0.99f, 0.99f, 0.99f, 0.99f, 0.99f, 0.99f, 1.00f
};

int		Offset  		= 0;
int   DeltaAngle  = 0;
short	LockedCount   = 0;
short LoseStepCount = 0;

void OS_TimerStop(unsigned char axis)
{
	switch(axis)
	{
		case Motor_X:
		{
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(&htim3, 0);  							//复位计数器为0
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
		}break;
		case Motor_Y:
		{
			
		}break;
		case Motor_Z:
		{
			
		}break;
		default: break;
	}
#if POSITION_SENSOR_EN
	HAL_TIM_Base_Stop_IT(&htim14);
	HAL_TIM_Base_Stop(&htim14);
	__HAL_TIM_SET_COUNTER(&htim14, 0);  							//复位计数器为0
	__HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
#endif
}

void OS_TimerStart(unsigned char axis)
{
	switch(axis)
	{
		case Motor_X:
		{
			__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
			__HAL_TIM_SET_COUNTER(&htim3, 0);  							//复位计数器为0
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 20);
			HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		}break;
		case Motor_Y:
		{
			
		}break;
		case Motor_Z:
		{
			
		}break;
		default: break;
	}
#if POSITION_SENSOR_EN
	__HAL_TIM_CLEAR_FLAG(&htim14, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim14, 0);  							//复位计数器为0
	HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start(&htim14);
#endif
}
/*******************************************************************************
函 数: void OS_SetPowerIO()
功 能: 电机PWM引脚电平输出设置
*******************************************************************************/
inline void OS_SetPowerIO(unsigned char axis, unsigned char m_level)
{
	switch(axis)
	{
		case Motor_X:
		{
			TMC2209_StepFreq_OnlyLevel(m_level);
		}break;
		case Motor_Y:
		{

		}break;
		case Motor_Z:
		{

		}break;
		default: break;
	}
}
/*******************************************************************************
函 数: void OS_Direction()
功 能: 电机运动方向控制
*******************************************************************************/
inline void OS_Direction(unsigned char axis, unsigned char m_dir)
{
	switch(axis)
	{
		case Motor_X:
		{
			TMC2209_MotorDir(m_dir);
		}break;
		case Motor_Y:
		{
			
		}break;
		case Motor_Z:
		{
			
		}break;
		default: break;
	}
}
/*******************************************************************************
函 数: void OS_LoadTimer()
功 能: 加载定时计数器频率，修改PWM的频率.
*******************************************************************************/
inline void OS_LoadTimer(unsigned char axis, float m_freq)
{
	switch(axis)
	{
		case Motor_X:
		{
			TMC2209_StepFreq_Set(ToPreiod(m_freq));
		}break;
		case Motor_Y:
		{
			
		}break;
		case Motor_Z:
		{
			
		}break;
		default: break;
	}
}
/*******************************************************************************
函 数: Stepper_Init(void)
功 能: 构造函数.完成变量的初始化
*******************************************************************************/
bool Stepper_Init(unsigned char PlsStopLvl)
{
	TMC2209_Init(); 
	
	for(unsigned char i = 0; i < Motor_Number; i++)
	{
		OS_TimerStop(i);
	}

	for(unsigned char i = 0; i < Motor_Number; i++)
	{
		Config[i].MotorAxis   	= i;
		Config[i].RunDir     		= DIR_ZHE;  				//电机的转动方向.
		Config[i].PlsPerCir  		= Pulse;        		//转一圈,脉冲数量.

		System[i].RemainSteps 	= 0;        				//目标步数.控制步进电机接下来运行多少步数.
		System[i].CurrentSteps 	= 0;        				//当前步数.
		System[i].State        	= MOTOR_STOP;
		System[i].CurFreq      	= 0;								//标记当前速度.
		System[i].ErrorCode  		= 0x00;

		Config[i].RPM          	= 0;        				//电机的转速.config.RPM.
		Config[i].TargetFrq    	= 0;        				//目标频率.Hz.
		Config[i].MinPlsFrq    	= DefaultMinFreq;		//脉冲引脚输出的脉冲最低频率.
		Config[i].SpreaadFrq    = Speed_Balance_Point * ((float)Config[i].PlsPerCir / 60.0f);
#if LIMIT_SWITCH_EN
#if LIMIT_HARD_SWITCH_EN
		Config[i].EnHardLsw    	= false;
#endif
		Config[i].EnSoftLsw    	= false;
		Config[i].SoftLswMax   	=  Pulse * 2;
		Config[i].SoftLswMin   	= -Pulse * 2;
#endif
#if POSITION_SENSOR_EN
		Config[i].TargetAngle   = 0;

		System[i].LastAngle     = 0;
		System[i].CurrentAngle  = 0;
		System[i].RemainAngle   = 0;
#endif
		Ramp[i].EnRamp     		 	= false;        		//速度规划使能标志.
		Ramp[i].RampMode   			= RAMP_MODE_L;  		//速度规划算法模式.
		Ramp[i].RampLength 			= 200;						  //速度规划加减速的长度.

		System[i].Location 			= 0;            		//全局变量.记录当前位置.
		Config[i].StopLevel 		= PlsStopLvl;  			//设置当步进电机不运行时,脉冲输出端的电平.
	}

	for(unsigned i = 0; i < Motor_Number; i++)
	{
		OS_SetPowerIO(i, Config[i].StopLevel);			//PWM引脚端口初始化.
		OS_Direction(i, Config[i].RunDir);					//方向引脚端口初始化.
		OS_TimerStart(i);
		Stepper_Stop(i);                         				//电机停止.
	}
#if POSITION_SENSOR_EN
	//USE_ABSOLUTE_ANGLE	USE_RELATIVE_ANGLE
	if(!MT6701_Init(USE_ABSOLUTE_ANGLE))
	{
		return false;
	}
	for(unsigned i = 0; i < Motor_Number; i++)
	{
		Stepper_ResetZero(i);
	}
#if PID_CONTROL_EN
	for(unsigned i = 0; i < Motor_Number; i++)
	{
		PID_Parameter_Init(i);
	}
#endif
#endif

	return true;
}
#if POSITION_SENSOR_EN
#if PID_CONTROL_EN
void PID_Parameter_Init(unsigned char axis)
{
	switch(axis)
	{
		case Motor_X:
		{
			PIDPara_Incr[0].Kp = 1;
			PIDPara_Incr[0].Ki = 0;
			PIDPara_Incr[0].Kd = 1;
			PIDPara_Incr[0].Error = 0;
			PIDPara_Incr[0].ErrorLast_1 = 0;
			PIDPara_Incr[0].ErrorLast_2	= 0;
			PIDPara_Incr[0].ValueMax		= 0;
			PIDPara_Incr[0].ValueMin		= 0;
		}break;
		case Motor_Y:
		{

		}break;
		case Motor_Z:
		{

		}break;
		default: break;
	}
}
#endif
void Stepper_ResetZero(unsigned char axis)
{
	unsigned char dir;
	
	Stepper_EnRamp(Motor_N, true);
	Stepper_SetRampMode(Motor_N, RAMP_MODE_L);	//RAMP_MODE_L RAMP_MODE_S
	Stepper_SetSpeed(Motor_N, 20);
	Stepper_SetMinSpeed(Motor_N, 5);	
	
	int diff_angle = 0;
	for (char i = 0; i < 10; i++)
	{
		diff_angle += Calibration_Angle_Read(Motor_N);
		HAL_Delay(100);
	}
	diff_angle = diff_angle / 10;
	if(diff_angle >= ENCODER_MAX_VALUE / 2)//大于180°
	{
		Offset = ENCODER_MAX_VALUE - diff_angle;
		dir    = DIR_FAN;
	}
	else
	{
		Offset = diff_angle;
		dir    = DIR_ZHE;
	}
	// if (Offset < (unsigned int)ENCODER_MAX_ERROR)
	// {
	// 	return;
	// }
	float step_length = (float)Offset * OneRoundStep * MicroStep / ENCODER_MAX_VALUE;
	//根据加减速的步进长度
	Stepper_SetRampLength(Motor_N, (unsigned long)(step_length * 0.25 + 0.5));
	Stepper_Run(Motor_N, dir, (unsigned long)(step_length + 0.5));
	Stepper_Wait(Motor_N);
	System[axis].Location = 0;
//	Stepper_WaitUntil(Motor_N, 6000);
}

bool isNeedCalibration()
{
	if (isCaliDataExist())
	{
		return false;
	}

	return true;
}

bool Stepper_Calibration(unsigned char axis)
{
/*归零，到达机械零点。但是没有物理反馈，无法知道是否真的到达机械零点
	所以实际上下面做的校准，没有太大意思。仅仅只是思路和逻辑*/
	Stepper_ResetZero(axis);
	int angle = Calibration_Angle_Read(Motor_N);
	if ((angle < ENCODER_MAX_VALUE - ANGLE_DIFF_THEO * 2 && angle > ENCODER_MAX_VALUE / 2) ||
			(angle > ANGLE_DIFF_THEO * 2 && angle < ENCODER_MAX_VALUE / 2))
	{
		return false;
	}
	__LOG__("already to zero position\n");
	HAL_Delay(1000);
	/*1. 纯软件的磁编码非线性补偿，只是为了解决传感器非线性误差*/
	if (!MT6701_INL_Calibration(axis, DIR_ZHE, DIR_FAN,
	                            OneRoundStep * MicroStep / CALIBRATION_RANGE, Stepper_Run))
	{
		return false;
	}
/*真正提高精度，需要机械角度能对上传感器角度。例如机械角度1.8°
	传感器读到的只有1.7°或者1.9°，加上补偿要变成1.8°和机械角度要对得上
	同样缺少物理反馈，无法知道真实物理角度 TODO*/

	return true;
}
#endif
/*******************************************************************************
函 数: void OS_NRamp(void)
功 能: 无加减速.
*******************************************************************************/
inline void OS_NRamp(unsigned char axis)
{
	System[axis].State   = MOTOR_RUN;  			//恒速运行
	System[axis].CurFreq = Config[axis].TargetFrq;

	OS_LoadTimer(axis, System[axis].CurFreq);
}
/*******************************************************************************
函 数: void OS_LRamp(void)
功 能: 直线加减速函数.
*******************************************************************************/
inline void OS_LRamp(unsigned char axis)
{
	//恒速运行
	if((System[axis].CurrentSteps > Ramp[axis].RampLength) && 
		 (System[axis].RemainSteps  > Ramp[axis].RampLength))
	{
		System[axis].State   = MOTOR_RUN;          
		System[axis].CurFreq = Config[axis].TargetFrq;
	
		OS_LoadTimer(axis, System[axis].CurFreq);
		
		return;
	}
	//加速
	if(System[axis].CurrentSteps <= System[axis].RemainSteps)
	{                             			//已经运行的步数<=剩余的步数.
		System[axis].State   = MOTOR_UP;  //运动的前半部分.加速过程中.
		System[axis].CurFreq = (unsigned long)(Config[axis].TargetFrq / Ramp[axis].RampLength) * //除0警告
                            System[axis].CurrentSteps +
                           (unsigned long)Config[axis].MinPlsFrq;
		if(System[axis].CurFreq > Config[axis].TargetFrq)
		{
			System[axis].CurFreq = Config[axis].TargetFrq;
		}
		if (System[axis].CurFreq > Config[axis].SpreaadFrq)
		{
			TMC2209_Spread(false);
		}
	}
	//减速
	else                      					//已经运行的步数>剩余的步数.
	{                             			//运动的后半部分.减速过程中.
		System[axis].State   = MOTOR_DOWN;
		System[axis].CurFreq = (unsigned long)(Config[axis].TargetFrq / Ramp[axis].RampLength) * //除0警告
													  System[axis].RemainSteps + 
													 (unsigned long)Config[axis].MinPlsFrq;
		if(System[axis].CurFreq > Config[axis].TargetFrq)
		{
			System[axis].CurFreq = Config[axis].TargetFrq;
		}
		if (System[axis].CurFreq < Config[axis].SpreaadFrq)
		{
			TMC2209_Spread(true);
		}
	}

	OS_LoadTimer(axis, System[axis].CurFreq);
}
/*******************************************************************************
函 数: void OS_SRamp(void)
功 能: S曲线加减速函数.
*******************************************************************************/
inline void OS_SRamp(unsigned char axis)
{
	//恒速阶段
	if((System[axis].CurrentSteps > Ramp[axis].RampLength) && 
		 (System[axis].RemainSteps  > Ramp[axis].RampLength))
	{                                       									//不在加减速的范围之内.
		System[axis].State   = MOTOR_RUN;        								//恒速运行
		System[axis].CurFreq = Config[axis].TargetFrq;
		
		OS_LoadTimer(axis, System[axis].CurFreq);
	
		return;
	}

	if(System[axis].CurrentSteps <= System[axis].RemainSteps )//已经运行的步数<=剩余的步数.
	{                                   											//运动的前半部分.加速过程中.
		System[axis].State = MOTOR_UP;       							
		unsigned long Index = 100 * System[axis].CurrentSteps / Ramp[axis].RampLength;	//除0警告
		if(Index >= 100) 
		{
			Index = 99;
		}
		System[axis].CurFreq = Config[axis].TargetFrq * SRampTable[Index];
		if(System[axis].CurFreq > Config[axis].TargetFrq)
		{
			System[axis].CurFreq = Config[axis].TargetFrq;
		}
		if(System[axis].CurFreq < Config[axis].MinPlsFrq)
		{
			System[axis].CurFreq = Config[axis].MinPlsFrq;
		}
		if (System[axis].CurFreq > Config[axis].SpreaadFrq)
		{
			TMC2209_Spread(false);
		}
	}
	else                                											//已经运行的步数>剩余的步数.
	{                                   											//运动的后半部分.减速过程中.
		System[axis].State = MOTOR_DOWN;
		unsigned long Index	= 100 * System[axis].RemainSteps / Ramp[axis].RampLength;		//除0警告
		if(Index >= 100) 
		{
			Index = 99;
		}
		System[axis].CurFreq = Config[axis].TargetFrq * SRampTable[Index];
		if(System[axis].CurFreq > Config[axis].TargetFrq)
		{
			System[axis].CurFreq = Config[axis].TargetFrq;
		}
		if(System[axis].CurFreq < Config[axis].MinPlsFrq)
		{
			System[axis].CurFreq = Config[axis].MinPlsFrq;
		}
		if (System[axis].CurFreq < Config[axis].SpreaadFrq)
		{
			TMC2209_Spread(true);
		}
	}
	
	OS_LoadTimer(axis, System[axis].CurFreq);
}
/*******************************************************************************
函 数: void Stepper_GetSpeed(void)
功 能: 获取步进电机设置的转速.
*******************************************************************************/
float Stepper_GetSpeed(unsigned char axis)
{
	return Config[axis].RPM;
}
/*******************************************************************************
函 数: float Stepper_GetCurSpeed(void)
功 能: 获取电机当前正在运行的速度.单位:RPM.
*******************************************************************************/
float Stepper_GetCurSpeed(unsigned char axis)
{
	return (System[axis].CurFreq * 60.0f / (float)Config[axis].PlsPerCir);
}
/*******************************************************************************
函 数: Uchar Stepper_CurDir(void)
功 能: 获取当前电机的运动方向
*******************************************************************************/
unsigned char Stepper_CurDir(unsigned char axis)
{
	return Config[axis].RunDir;
}
/*******************************************************************************
函 数: void Stepper_Run(void)
功 能: 步进电机转动指令.命令步进电机转动一定的步数.
参 数: myDir   -- 步进电机转动方向. 0/1.
       mySteps -- 步进电机转动的步数.
举 例: Stepper_Run(DIR_ZHENG, 500); 代表往正方向运行500个脉冲.过程自动加减速.
*******************************************************************************/
void Stepper_Run(unsigned char axis, unsigned char m_dir, unsigned long m_steps)
{
	if(m_steps == 0 || Stepper_ErrorStatus(axis,MOTOR_HARD_LIMIT_L|MOTOR_HARD_LIMIT_R|
																									 MOTOR_LOSE_STEP|MOTOR_LOCKED) != 0x00)
	{
		return;
	}
	
	Config[axis].RunDir 		 	= m_dir;     				//记录运动方向.
	OS_Direction(axis, Config[axis].RunDir);      //将运动方向输出到引脚.
#if LIMIT_SWITCH_EN
	if (Config[axis].EnSoftLsw)
	{
		if (Config[axis].RunDir == DIR_ZHE)
		{
			if (System[axis].Location >= Config[axis].SoftLswMax)//正转.最大值限制
			{
				// System[axis].State = MOTOR_ALARM;
				System[axis].ErrorCode |= MOTOR_SOFT_LIMIT_L;
				return;
			}
			m_steps = Config[axis].SoftLswMax - System[axis].Location < m_steps ?
								Config[axis].SoftLswMax - System[axis].Location : m_steps;
		}
		else
		{
			if (System[axis].Location <= Config[axis].SoftLswMin)//反转.最大值限制
			{
				// System[axis].State = MOTOR_ALARM;
				System[axis].ErrorCode |= MOTOR_SOFT_LIMIT_R;
				return;
			}
			m_steps = System[axis].Location - Config[axis].SoftLswMin < m_steps ?
								System[axis].Location - Config[axis].SoftLswMin : m_steps;
		}
		Stepper_ErrorClear(Motor_N, MOTOR_SOFT_LIMIT_R | MOTOR_SOFT_LIMIT_L);
	}
#endif
	System[axis].RemainSteps  = m_steps;      		//目标步数.两个中断为一个脉冲.
	System[axis].CurrentSteps = 0;            		//当前步数.
	System[axis].CurFreq 		  = Config[axis].TargetFrq; 
	//不管是否启用加减速.第一次中断频率越快越好.快速进入中断.                                        
	//在scan()函数内会计算加减速
	System[axis].State  		 	= MOTOR_UP;         //状态:加速
#if POSITION_SENSOR_EN
	Config[axis].TargetAngle  = ENCODER_MAX_VALUE * m_steps / (OneRoundStep * MicroStep);
	System[axis].RemainAngle  = Config[axis].TargetAngle;
	System[axis].LastReAngle  = System[axis].RemainAngle;
	System[axis].CurrentAngle = Calibration_Angle_Read(Motor_N);
	if(Config[axis].RunDir == DIR_ZHE)
	{
		System[axis].LastAngle  = System[axis].CurrentAngle + (ENCODER_MAX_VALUE / Pulse);
	}
	else
	{
		System[axis].LastAngle  = System[axis].CurrentAngle - (ENCODER_MAX_VALUE / Pulse);
	}
	LoseStepCount = 0;
#if	PID_CONTROL_EN
	PIDPara_Incr[axis].ErrorLast_1 = 0;
	PIDPara_Incr[axis].ErrorLast_2 = 0;
#endif
#endif
	OS_LoadTimer(axis, System[axis].CurFreq);    	//设置定时计数器频率.
	OS_TimerStart(axis);
}
/*******************************************************************************
函 数: void Stepper_OneAxisMove(void)
功 能: 电机运动到指定全局位置.差量运行,从当前位置运行到目标位置.
距 离: 例如当前电机处于坐标100脉冲处,Stepper_MoveTo(200)以后,电机会往正方向运行100脉冲.
*******************************************************************************/
void Stepper_MoveTo1(unsigned char axis, long x)
{
	long CurLocation = Stepper_GetPosition(axis);	//获取电机的当前坐标

	if(CurLocation == x)													//当前已经在目标点.不需要运动.
	{
		return;
	}

	if(x > CurLocation)														//目标点在当前点的正方向.
	{
		Stepper_Run(axis, DIR_ZHE, x - CurLocation);
	}
	else																					//目标点在当前点的负方向.
	{
		Stepper_Run(axis, DIR_FAN,   CurLocation - x);
	}
}
#if  (Motor_Number == 2)
/*******************************************************************************
函 数: void MoveTo(void)
功 能: 电机运动到指定全局位置.
距 离: 例如当前位置是(100,200),当执行MoveTo(300,800)时,X轴电机往正方向移动200脉冲,
       Y轴电机往正方向移动600脉冲.
*******************************************************************************/
void Stepper_MoveTo2(long x, long y)
{
	unsigned char dirx, diry, min_one, max_one;
    
	long delta_x = x - System[Motor_X].location;    //X轴变化值
	long delta_y = y - System[Motor_Y].location;    //Y轴变化值
	
	unsigned long abs_x = (delta_x>0)?(delta_x):(-delta_x); 
	unsigned long abs_y = (delta_y>0)?(delta_y):(-delta_y);
	
	float k = (abs_x > abs_y) ? (abs_y / abs_x) : (abs_x / abs_y);

	if( delta_x > 0 )
		dirx = DIR_ZHE;
	else
		dirx = DIR_FAN;
	if( delta_y > 0 )
		diry = DIR_ZHE;
	else
		diry = DIR_FAN;
	
	if(delta_x > delta_y)
	{
		max_one = Motor_X;
		min_one = Motor_Y;
	}
	else
	{
		max_one = Motor_Y;
		min_one = Motor_X;
	}
	/*步长大的转速不需要变,步长小的需要限制加速距离和转速
	  重新计算, 但总体肯定要比步长大的要小。
		怎么算有很大的优化空间嗷!!!
	*/
	unsigned long ramp_length = Ramp[max_one].RampLength * k;
	float speedm = Config[max_one].RPM * k;
	
	Stepper_SetRampLength(min_one, ramp_length);
	Stepper_SetSpeed(min_one, speedm);
	
	Stepper_Run(Motor_X, dirx, abs_x);
	Stepper_Run(Motor_Y, diry, abs_y);
}
#elif(Motor_Number > 2)
/*******************************************************************************
函 数: void MoveTo(void)
功 能: 电机运动到指定全局位置.
距 离: 例如当前位置是(100,200,300),当执行MoveTo(300,800,1000)时,
			 X轴电机往正方向移动200脉冲,
       Y轴电机往正方向移动600脉冲,
			 Z轴电机往正方向移动700脉冲
*******************************************************************************/
void Stepper_MoveTo3(long x, long y, long z)
{
	unsigned char dirx,diry,dirz;
	
	long x_cur_location = System[Motor_X].location;
	long y_cur_location = System[Motor_Y].location;
	long z_cur_location = System[Motor_Z].location;
    
	long delta_x = x - x_cur_location;    //X轴变化值
	long delta_y = y - y_cur_location;    //Y轴变化值
	long delta_z = z - z_cur_location;    //Z轴变化值

	unsigned long abs_x = (delta_x>0)?(delta_x):(-delta_x); 
	unsigned long abs_y = (delta_y>0)?(delta_y):(-delta_y);
	unsigned long abs_Z = (delta_z>0)?(delta_z):(-delta_z);
	
	float k1 = ;
	float k2 = 

	if( delta_x > 0 )
		dirx = DIR_ZHENG;
	else
		dirx = DIR_FAN;
	if( delta_y > 0 )
		diry = DIR_ZHENG;
	else
		diry = DIR_FAN;
	if( delta_z > 0 )
		dirz = DIR_ZHENG;
	else
		dirz = DIR_FAN;
	/*要做转速加速距离等限制才能实现直线
	即多个轴同时到达目的地。*/
	
	Stepper_Run(Motor_X, dirx, delta_x);
	Stepper_Run(Motor_Y, dirx, delta_y);
	Stepper_Run(Motor_Z, dirz, delta_z);
}
#endif
/*DDA插补*/
/*......*/
/*TODO*/
/*******************************************************************************
函 数: long Stepper_GetPosition(void)
功 能: 获取电机的全局位置.
*******************************************************************************/
long Stepper_GetPosition(unsigned char axis)
{
	return System[axis].Location;
}
/*******************************************************************************
函 数: long Stepper_SetPosition(void)
功 能: 设置电机的全局位置.
*******************************************************************************/
void Stepper_SetPosition(unsigned char axis, long pos)
{
	System[axis].Location = pos;
}
/*******************************************************************************
函 数: void Stepper_Wait(void)
功 能: 一直等待,直到电机运行停止.阻塞函数,尽量不用.注意电机报警死锁.
*******************************************************************************/
void Stepper_Wait(unsigned char axis)
{
	while(System[axis].State != MOTOR_STOP)
	{
		if (System[axis].ErrorCode != MOTOR_NORMAL)
		{
			return;
		}
		//KickDog();
	}
}
//带超时的等待电机停止
void Stepper_WaitUntil(unsigned char axis, unsigned long overTime)
{
	unsigned int curTime = HAL_GetTick();

	for(;;)
	{
		//电机停止后退出
		if(System[axis].State == MOTOR_STOP)
		{
			return;
		}
		//计算等待时间
		if((HAL_GetTick() - curTime) >= overTime)
		{
			return;
		}
	}
}
/*******************************************************************************
函 数: void Stepper_Stop(void)
功 能: 立即停止步进电机的运行.关闭定时器.
*******************************************************************************/
void Stepper_Stop(unsigned char axis)
{
/*
	电机停止时输出电平，实际没用。高低取决于定时器
	关闭的瞬间PWM的状态，在关闭定时器的时候就已经
	决定好了。但是可以作为下一次PWM开启做准备
*/
	OS_SetPowerIO(axis, Config[axis].StopLevel);		//重新装载CCR比较寄存器
	OS_TimerStop(axis);
	
	System[axis].CurFreq 			= 0;              		//当前速度.
	System[axis].State  			= MOTOR_STOP;     		//电机状态:停止.    

	System[axis].RemainSteps  = 0;
	System[axis].CurrentSteps = 0;

	switch(axis)
	{
		case Motor_X:
		{
			TMC2209_Spread(true);
		}break;
		case Motor_Y:
		{

		}break;
		case Motor_Z:
		{

		}break;
		default: break;
	}
}
/*******************************************************************************
函 数: void Stepper_RStop(void)
功 能: 带速度规划的减速停止步进电机.适用于运行中缓慢停止电机.停止距离取决RampLength.
*******************************************************************************/
void Stepper_RStop(unsigned char axis)
{
	if(System[axis].CurFreq == 0)    								//当前电机是停止的.
	{
		return;
	}

	if(!Ramp[axis].EnRamp)      										//速度规划没有被启用.
	{
		Stepper_Stop(axis);
		
		return;
	}
	
	if(System[axis].CurrentSteps <= System[axis].RemainSteps)	//运动的前半部分     
	{										
		if(System[axis].CurrentSteps < Ramp[axis].RampLength)		//正在加速阶段.    
		{
			System[axis].RemainSteps = System[axis].CurrentSteps;
		}
		else
		{
			System[axis].RemainSteps = Ramp[axis].RampLength;
		}
	}
	else                            													//运动的后半部分
	{
		if(System[axis].RemainSteps < Ramp[axis].RampLength)		//已经在减速阶段了.
		{    
																														//不需要处理.
		}
		else
		{
			System[axis].RemainSteps = Ramp[axis].RampLength;
		}
	}
}
#if LIMIT_SWITCH_EN
#if LIMIT_HARD_SWITCH_EN
/*******************************************************************************
函 数: void Stepper_HardLimit_En(void)
功 能: 开启/关闭硬件限位开关功能.
*******************************************************************************/
void Stepper_HardLimit_En(unsigned char axis, bool enable)
{
	if(enable)
	{
		Config[axis].EnHardLsw  = true;
	}
	else
	{
		Config[axis].EnHardLsw  = false;
	}
}
// 读取硬件限位的使能状态
bool Stepper_HardLimitStatus(unsigned char axis)
{
	return Config[axis].EnHardLsw;
}
#endif
/*******************************************************************************
函 数: void Stepper_SoftLimit_En(void)
功 能: 开启/关闭软件限位开关功能.限制全局坐标的最大值和最小值.
*******************************************************************************/
void Stepper_SoftLimit_En(unsigned char axis, bool enable)
{
	if(enable)
	{
		Config[axis].EnSoftLsw 	= true;
	}
	else
	{
		Config[axis].EnSoftLsw	= false;
	}
}
// 读取软件限位的使能状态
bool Stepper_SoftLimitStatus(unsigned char axis)
{
	return Config[axis].EnSoftLsw;
}
/*******************************************************************************
函 数: void Stepper_SoftLimitMin(void)
功 能: 设置软件限位的最小值
*******************************************************************************/
void Stepper_SoftLimitMin(unsigned char axis, long MinValue)
{
	Config[axis].SoftLswMin = MinValue;
}
/*******************************************************************************
函 数: void Stepper_SoftLimitMax(void)
功 能: 设置软件限位的最大值
*******************************************************************************/
void Stepper_SoftLimitMax(unsigned char axis, long MaxValue)
{
	Config[axis].SoftLswMax = MaxValue;
}
#if LIMIT_HARD_SWITCH_EN
/*******************************************************************************
函 数: bool HardLSW_OS_Limit(void)
功 能: 硬件限位判断函数.内部函数.
*******************************************************************************/
inline bool HardLSW_OS_Limit(unsigned char axis)
{
	if(!Config[axis].EnHardLsw)             			//限位功能被关闭.
	{
		return false;
	}
	
	if(Config[axis].RunDir == DIR_ZHE)    				//电机正转.
	{
		if(ReadLimitIOL() == 1)
		{                             							//正转.增量限位开关动作.
			System[axis].ErrorCode |= MOTOR_HARD_LIMIT_L; 		
			
			return true;
		}
	}
	else                            							//电机反转.
	{
		if(ReadLimitIOR() == 1)
		{                           								//反转.负方向限位动作.
			System[axis].ErrorCode |= MOTOR_HARD_LIMIT_R;
			
			return true;
		}
	}

	return false;
}
#endif
// /*******************************************************************************
// 函 数: bool SoftLSW_OS_Limit(void)
// 功 能: 软件限位判断函数.内部函数.
// *******************************************************************************/
// inline bool SoftLSW_OS_Limit(unsigned char axis)
// {
// 	if(!Config[axis].EnSoftLsw)         					//关闭功能
// 	{
// 		return false;
// 	}
//
// 	if(Config[axis].RunDir == DIR_ZHE)   					//电机正转.
// 	{
// 		if(System[axis].Location > Config[axis].SoftLswMax )
// 		{                           								//正转.最大值限制.
// 			System[axis].ErrorCode |= MOTOR_SOFT_LIMIT_L;
//
// 			return true;
// 		}
// 	}
// 	else                            							//电机反转.
// 	{
// 		if(System[axis].Location < Config[axis].SoftLswMin)
// 		{                           								//反转.最小值限制.
// 			System[axis].ErrorCode |= MOTOR_SOFT_LIMIT_R;
//
// 			return true;
// 		}
// 	}
//
// 	return false;
// }
#endif
char Stepper_ErrorStatus(unsigned char axis, unsigned char error)
{
	return (System[axis].ErrorCode & error);
}

void Stepper_ErrorClear(unsigned char axis, unsigned char error)
{
	System[axis].ErrorCode &= (~error); 
}
/*******************************************************************************
函 数: Uchar Stepper_GetState(void)
功 能: 获取当前电机的运行状态.
返 回: 
MOTOR_STOP      电机停止中
MOTOR_UP        电机加速中
MOTOR_RUN       电机恒速中
MOTOR_DOWN      电机减速中
MOTOR_ALARM     报警
*******************************************************************************/
char Stepper_GetState(unsigned char axis)
{
	return System[axis].State;
}
/*******************************************************************************
函 数: void Stepper_Scan(void)
功 能: 这个函数是由定时器中断调用的函数.
*******************************************************************************/
void Stepper_Scan(unsigned char axis)
{
//	//-----------------
//	//电机已经运行结束
//	//步进电机已经运行完毕设置的步数.
//	if(System[axis].RemainSteps == 0)
//	{
//		Stepper_Stop(axis);
//		
//		return;
//	}
#if POSITION_SENSOR_EN
	System[axis].CurrentAngle = Calibration_Angle_Read(Motor_N);
	if(Config[axis].RunDir == DIR_FAN)//0
	{
		DeltaAngle = System[axis].CurrentAngle - System[axis].LastAngle;
	}
	else															//1
	{
		DeltaAngle = System[axis].LastAngle - System[axis].CurrentAngle;
	}
	if(DeltaAngle < -(ENCODER_MAX_VALUE / 2))	//转速在大于3000r/min,10ms下也会发生小于-180°的情况
	{
		DeltaAngle = DeltaAngle + ENCODER_MAX_VALUE;
	}
	else if(DeltaAngle >= 0)
	{

	}
	else
	{
		DeltaAngle = 0;
	}
	System[axis].RemainAngle -= DeltaAngle;
	System[axis].LastAngle   =  System[axis].CurrentAngle;
#endif
	//-----------------
	//速度规划函数
	if(Ramp[axis].EnRamp)                     //开启速度规划.
	{
		if(Ramp[axis].RampMode == RAMP_MODE_S)	//S曲线加减速.
		{
			OS_SRamp(axis);
		}
		else
		{
			OS_LRamp(axis);
		}
	}
	else                          						//关速度规划.
	{
		OS_NRamp(axis);
	}
#if LIMIT_HARD_SWITCH_EN
	//硬件限位
	if(HardLSW_OS_Limit(axis))      					//硬件限位发生
	{
		// System[axis].State = MOTOR_ALARM;		//换一种
	
		return;/*----------------------------------------!!!!!*/
	}
#endif
	//运行步数计算
	if(System[axis].RemainSteps > 0)     			//剩余步数
	{
		System[axis].RemainSteps--;
	}		
	System[axis].CurrentSteps++;         			//步进电机已经运行的步数.
	if(Config[axis].RunDir == DIR_ZHE)				//记录全局位置.
	{
		System[axis].Location++;
	}
	else
	{
		System[axis].Location--;
	}
#if POSITION_SENSOR_EN
	if ((System[axis].State == MOTOR_DOWN && System[axis].CurFreq <= Config[axis].SpreaadFrq)
			|| !Ramp[axis].EnRamp)	//没有加减速其实可以不补偿，没意义了，速度稍微高一点运动必然出问题
	{
		if(System[axis].CompensationPulse != 0)
		{
			int comp_dir = System[axis].CompensationPulse > 0 ? 1 : -1;
			// 补偿步数（不影响速度规划，因为规划已执行完毕）
			System[axis].RemainSteps += comp_dir;
			// if(comp_dir > 0)
			// {
			// 	System[axis].RemainSteps++;  // 正补偿：增加剩余步数
			// }
			// else if(System[axis].RemainSteps > 0)
			// {
			// 	System[axis].RemainSteps--;  // 负补偿：减少剩余步数
			// }
			System[axis].CurrentSteps += comp_dir;
			System[axis].CompensationPulse -= comp_dir;
		}
	}
#endif
	//-----------------
	//电机已经运行结束
	//步进电机已经运行完毕设置的步数.
	if(System[axis].RemainSteps <= 0)
	{
		Stepper_Stop(axis);
	}
}
#if POSITION_SENSOR_EN
volatile int temp_angle = 0;
void Stepper_Position_Scan(unsigned char axis)
{
	if (System[axis].State != MOTOR_STOP)
	{
		// printf("angle %d %d, count %d %d\n", System[axis].LastReAngle,
		// 																		 System[axis].RemainAngle,
		// 																		 LockedCount, LockedCount);
		temp_angle = System[axis].LastReAngle - System[axis].RemainAngle;
		if(temp_angle <= LockedThreshold)
		{
			LockedCount++;
		}
		else
		{
			LockedCount--;
			LockedCount = LockedCount & ~(LockedCount >> 15);//(sizeof(short) - 1)
		}
		if(System[axis].State == MOTOR_RUN)				//目前仅在有匀速运动的时候进行判断
		{
			if(temp_angle > (int)LoseStepThreshold(Config[axis].RPM) / 2)
			{
				LoseStepCount--;
				LoseStepCount = LoseStepCount & ~(LoseStepCount >> 15);//(sizeof(short) - 1)
			}
			else
			{
				LoseStepCount++;
			}
		}
		if (LockedCount >= LockedCountMax)
		{
			// System[axis].State = MOTOR_ALARM;
			System[axis].ErrorCode |= MOTOR_LOCKED;		//MOTOR_LOCKED
			return;
		}
		if(LoseStepCount >= LoseStepCountMax)
		{
			// System[axis].State = MOTOR_ALARM;
			System[axis].ErrorCode |= MOTOR_LOSE_STEP;//MOTOR_LOSE_STEP
			return;
		}
		System[axis].LastReAngle = System[axis].RemainAngle;
	}
#if	PID_CONTROL_EN
	// if (System[axis].State == MOTOR_DOWN || !Ramp[axis].EnRamp)
	// {
	unsigned int RealPulseAngle = Pulse * (Config[axis].TargetAngle - System[axis].RemainAngle) /
	                              ENCODER_MAX_VALUE + 0.5;
	int error = (int)System[axis].CurrentSteps - (int)RealPulseAngle;
	if (error >= DynamicLagAngle || error <= -DynamicLagAngle)
	{
		if (error < 0)
		{
			PIDPara_Incr[axis].Error = error + DynamicLagAngle;
		}
		else
		{
			PIDPara_Incr[axis].Error = error - DynamicLagAngle;
		}
		int delta_value = PIDCompute_Incr(&PIDPara_Incr[axis], false);
		printf("%d, %d, %d\n", System[axis].CurrentSteps, RealPulseAngle, delta_value);
		System[axis].CompensationPulse += delta_value;
	}
	// }
#endif
}
#endif
/*******************************************************************************
函 数: void Stepper_EnRamp()
功 能: 开启/关闭 速度规划算法.
*******************************************************************************/
void Stepper_EnRamp(unsigned char axis, bool enable)
{
	if(enable)
	{
		Ramp[axis].EnRamp = true;
	}
	else
	{
		Ramp[axis].EnRamp = false;
	}
	Ramp[axis].RampLength = 1;
}
/*******************************************************************************
函 数: void Stepper_SetRampMode()		 RAMP_MODE_S    RAMP_MODE_L
功 能: 设置速度规划算法模式.(0--S曲线加减速;1--直线加减速)
*******************************************************************************/
void Stepper_SetRampMode(unsigned char axis, unsigned char mode)
{
	Ramp[axis].RampMode = mode;
}
/*******************************************************************************
函 数: void Stepper_SetRampLength()
功 能: 设置加减速的步进长度.
*******************************************************************************/
void Stepper_SetRampLength(unsigned char axis, unsigned long m_RampLength)
{
	if(m_RampLength <= 0)
	{
		m_RampLength = 0;
	}
	Ramp[axis].RampLengthSET = m_RampLength;
	// 以300RPM转速为界限作为比例下的的加减速长度，要根据当前的速度,设置对应的实际加减速长度
	Ramp[axis].RampLength 	 = Ramp[axis].RampLengthSET * (Config[axis].RPM / 300);
	Ramp[axis].RampLength   += 1;	//限制最小值，防止除0导致出问题
}
/*******************************************************************************
函 数: void Stepper_SetSpeed()
功 能: 设置步进电机的转速,单位RPM.
*******************************************************************************/
void Stepper_SetSpeed(unsigned char axis, float rpm)
{

	if(rpm <= 0 ||
		 Stepper_ErrorStatus(axis,MOTOR_HARD_LIMIT_L|MOTOR_HARD_LIMIT_R|
		 															 MOTOR_LOSE_STEP|MOTOR_LOCKED) != 0x00)
	{
		Stepper_Stop(axis);         	//停止电机.
		Config[axis].RPM		   = 0;
		Config[axis].TargetFrq = 0;
		
		return;
	}
	// if(rpm == Config[axis].RPM )  	//要设置的速度值没有发生变化.
	// {
	// 	return;
	// }
	if(!Ramp[axis].EnRamp)                     //开启速度规划.
	{
		if (rpm > Speed_Balance_Point)
		{
			TMC2209_Spread(false);
		}
		else
		{
			TMC2209_Spread(true);
		}
	}
	Config[axis].RPM       = rpm;   		//转速 RPM
	Config[axis].TargetFrq = Config[axis].RPM * ((float)Config[axis].PlsPerCir / 60.0f);
	//用户设置的RampLength(加减速长度),是300RPM转速下的加减速长度
	//要根据当前的速度,设置对应的实际加减速长度
	Ramp[axis].RampLength  = Ramp[axis].RampLengthSET * Config[axis].RPM / 300;
}
/*******************************************************************************
函 数: void Stepper_SetMinSpeed(void)
功 能: 设置电机加减速阶段的起步频率.电机运行的最小速度.
参 数: config.RPM--步进电机起步频率.
*******************************************************************************/
void Stepper_SetMinSpeed(unsigned char axis, float rpm)
{
	float PlsFrq = rpm * ((float)Config[axis].PlsPerCir / 60.0f) ;
	
	if(PlsFrq > 0)    		//最小速度大于零
	{
		Config[axis].MinPlsFrq = PlsFrq;
	}
	else            			//最低速度不能为零.不正确.
	{
		Config[axis].MinPlsFrq = DefaultMinFreq;    
	}
}
