/*******************************************************************************
    用于驱动步进电机或伺服电机.输出信号分为脉冲信号Pls和方向引脚Dir.
必须接步进电机驱动器或伺服电机驱动器.如果你是使用步进驱动器的话,那么占空比没啥用,
因为步进驱动器只关心你的频率,不关心你的占空比;PWM在直接驱动的时候,占空比才有用;   
*******************************************************************************/
#ifndef  __STEPPER_CONTROL_H_
#define  __STEPPER_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>
#include "TMC2209.h"

#define  	LOW											0
#define  	HIGH										1

#define  	DIR_ZHE	              	MotorFWD  //正转: 1
#define  	DIR_FAN                	MotorREV 	//反转: 0

#define  	RAMP_MODE_S            	0   			//S曲线加减速.
#define  	RAMP_MODE_L            	1   			//直线加减速.

#define   Motor_Number						1
#define   Motor_N									0
#define		Motor_X									Motor_N
#define 	Motor_Y									1			
#define   Motor_Z									2

#define		MOTOR_NORMAL						0
#define   MOTOR_ERROR_ALL					0xFF
#define   MOTOR_HARD_LIMIT_L		 (1 << 0)		//硬件左限位
#define		MOTOR_HARD_LIMIT_R		 (1 << 1)		//硬件右限位
#define   MOTOR_SOFT_LIMIT_L		 (1 << 2)		//软件左限位
#define   MOTOR_SOFT_LIMIT_R		 (1 << 3)		//软件右限位	
#define 	MOTOR_LOSE_STEP				 (1 << 4)		//丢步
#define   MOTOR_LOCKED					 (1 << 5)		//过载

/*----------HARD/SOFT LIMIT CONFIGURATION----------*/
#define		LIMIT_SWITCH_EN					1
#if LIMIT_SWITCH_EN
#define	  LIMIT_HARD_SWITCH_EN    0
#endif
/*----------POSITION SENSOR CONFIGURATION----------*/
#define  	POSITION_SENSOR_EN			1
#define   PID_CONTROL_EN					1

typedef enum
{
  MOTOR_STOP,                								//电机停止中.
  MOTOR_UP,                  								//电机加速中.
  MOTOR_RUN,                 								//电机恒速中.
  MOTOR_DOWN,                								//电机减速中.
  MOTOR_ALARM,               								//报警
}MotorStatus;

/*--------------------------------------------------------------------------------*/

struct STRUCT_RAMP
{
	unsigned char  	EnRamp;           				//速度规划使能标志.
	unsigned char  	RampMode;         				//速度规划算法模式.
	unsigned long   RampLength;       				//速度规划算法函数--加速的长度.  
	unsigned long   RampLengthSET;    				//300RPM为比例分界线的加减速长度
};

struct STRUCT_SYSTEM
{
	volatile char						State;         		//电机运行状态(加减速状态).
	volatile char						ErrorCode;
	volatile long   				Location;		  		//记录当前位置.
	volatile float  				CurFreq;       		//记录当前速度.
	volatile unsigned long	RemainSteps;			//剩余步数.
	volatile unsigned long	CurrentSteps;			//当前步数.
#if POSITION_SENSOR_EN
	volatile int						LastReAngle;
	volatile int   					RemainAngle;
	volatile int						LastAngle;
	volatile int						CurrentAngle;
#if PID_CONTROL_EN
	volatile int						CompensationPulse;
#endif
#endif
};

struct STRUCT_CONFIG
{
	unsigned int    PlsPerCir;        				//步进电机转动一圈,所需要的脉冲数量.	
	unsigned char   MotorAxis;								//
	unsigned char   StopLevel;        				//当步进电机停止运转时,输出的电平;默认高电平.
	unsigned char   RunDir;		        				//电机的转动方向.
	float   				RPM;              				//电机的转速._RPM.    
	float   				TargetFrq;        				//目标频率.
	float   				MinPlsFrq;        				//脉冲引脚输出的脉冲最低频率.
	float           SpreaadFrq;								//斩波开关界限频率点
#if LIMIT_SWITCH_EN
#if LIMIT_HARD_SWITCH_EN
	bool   					EnHardLsw;								//使能硬件限位
#endif
	bool	   				EnSoftLsw;        				//使能软件限位
	long   		 			SoftLswMax;       				//软件限位.最大值.
	long    				SoftLswMin;       				//软件限位.最小值.
#endif
#if POSITION_SENSOR_EN
	int							TargetAngle;
#endif
};

/*------------------------------------------------------------------------------------------*/

bool  Stepper_Init(unsigned char PlsStopLvl);	  					//电机变量初始化.
#if POSITION_SENSOR_EN
void  Stepper_ResetZero(unsigned char axis);
bool  isNeedCalibration();
bool  Stepper_Calibration(unsigned char axis);
void	Stepper_Position_Scan(unsigned char axis);
#endif
void  Stepper_EnRamp(unsigned char axis, bool enable);	  //开启/关闭加减速运动控制算法.
void  Stepper_SetRampMode(unsigned char axis,
													unsigned char mode);          	//设置加减速运动控制算法模式.
void  Stepper_SetRampLength(unsigned char axis,
														unsigned long m_RampLength);  //设置加减速的步进长度
void  Stepper_SetSpeed(unsigned char axis, float rpm);    //设置运行时最大转速.单位:RPM.
void  Stepper_SetMinSpeed(unsigned char axis, float rpm); //设置系统的最低转速.加减速的时候起作用.
float Stepper_GetSpeed(unsigned char axis);								//获取设置的速度
float Stepper_GetCurSpeed(unsigned char axis);           	//获取当前速度.单位:RPM.
unsigned char Stepper_CurDir(unsigned char axis);        	//获取当前运动方向.
void  Stepper_Run(unsigned char axis, 
									unsigned char m_dir, 
									unsigned long m_steps); 
void  Stepper_MoveTo1(unsigned char axis, long x);				//移动到目标全局位置.
#if  (Motor_Number ==2)
void 	Stepper_MoveTo2(long x, long y);
#elif(Motor_Number > 2)
void 	Stepper_MoveTo3(long x, long y, long z);
#endif
long  Stepper_GetPosition(unsigned char axis);						//获取当前位置.
void  Stepper_Wait(unsigned char axis);										//等待电机停止.
void  Stepper_WaitUntil(unsigned char axis, 
												unsigned long OverTime);       		//带超时的等待.
void  Stepper_Stop(unsigned char axis);                   //立即无条件停止电机. 
void  Stepper_RStop(unsigned char axis);                  //非立刻停止,带速度规划,推荐使用
#if LIMIT_SWITCH_EN
#if LIMIT_HARD_SWITCH_EN
void  Stepper_HardLimit_En(unsigned char axis,
													 bool enable);                  //开启/关闭硬件限位
bool  Stepper_HardLimitStatus(unsigned char);             //读取硬件限位的使能状态
#endif
void  Stepper_SoftLimit_En(unsigned char axis, 
													 bool enable);                  //开启/关闭软件限位
bool  Stepper_SoftLimitStatus(unsigned char axis);        //读取软件限位的使能状态
void  Stepper_SoftLimitMin(unsigned char axis, 
													 long MinValue);       					//设置软件限位的最小值
void  Stepper_SoftLimitMax(unsigned char axis, 
											     long MinValue);       					//设置软件限位的最大值
#endif
char  Stepper_ErrorStatus(unsigned char axis, unsigned char error);
void 	Stepper_ErrorClear(unsigned char axis, unsigned char error);
void  Stepper_Scan(unsigned char axis);										//步进电机驱动函数.在ISR中运行,并非用户调用.	
char  Stepper_GetState(unsigned char axis);								//获取电机运行状态.
  
#ifdef __cplusplus
}
#endif

#endif
