#include "m_task.h"
#include "Common.h"
#include "gpio.h"
#include "stm32g0xx_hal.h"

volatile bool isMotorReset = false;
volatile bool isSensorCali = false;
unsigned char CommunicationMode = UART_COMMUNICATION;
unsigned char DeviceStatus = DEVICE_PREPARING;
/*输入寄存器内容
*/
extern uint16_t usRegInputBuf[10];
/*保持寄存器内容
*/
extern uint16_t usRegHoldingBuf[10];
/*输出线圈状态
*/
extern uint8_t 	ucRegCoilsBuf[2];
/*离散输入状态
*/
extern uint8_t 	usRegDiscreteBuf[2];

os_task test_task;

void ErrorTask(void);
void BlockingTask(void);
void TestTask(os_task* task_t);

void m_OSKernelInit(void)
{
	test_task.task_id   	= 0;
	// test_task.task_flag 	= 255;
	test_task.task_flag 	= 1;
	test_task.interval_ms = 1000;
}

void m_OSKernelStart()
{
	ErrorTask();
	BlockingTask();
	TestTask(&test_task);
}

void StepperErrorHandle()
{
	char errorcode = 0;
#if POSITION_SENSOR_EN
	//触发过载、堵转或超速时，立即停机并报警，防止硬件损坏
	errorcode = Stepper_ErrorStatus(Motor_N, MOTOR_LOCKED | MOTOR_LOSE_STEP);
	switch(errorcode)
	{
		case MOTOR_LOSE_STEP :
		{
			// Stepper_ErrorClear(Motor_N, MOTOR_LOCKED);
			HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, GPIO_PIN_SET);
			Stepper_Stop(Motor_N);
		}break;
		case MOTOR_LOCKED :
		{
			// Stepper_ErrorClear(Motor_N, MOTOR_LOSE_STEP);
			HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, GPIO_PIN_SET);
			Stepper_Stop(Motor_N);
		}break;
		default : break;
	}
#endif
#if LIMIT_HARD_SWITCH_EN
	//触发硬件限位时，立即停机并报警，防止硬件损坏
	errorcode = Stepper_ErrorStatus(Motor_N, MOTOR_HARD_LIMIT_L | MOTOR_HARD_LIMIT_R);
	switch(errorcode)
	{
		case MOTOR_HARD_LIMIT_L :
		{
			//Stepper_Stop(Motor_N);
		}break;
		case MOTOR_HARD_LIMIT_R :
		{
			//Stepper_Stop(Motor_N);
		}break;
		default : break;
	}
	// Stepper_ErrorClear(Motor_N, MOTOR_HARD_LIMIT_L | MOTOR_HARD_LIMIT_R);
#endif
}

void StepperWarningHandle()
{
	char errorcode = 0;
#if LIMIT_SWITCH_EN
	//触发软件限位时，进行警告
	errorcode = Stepper_ErrorStatus(Motor_N, MOTOR_SOFT_LIMIT_L | MOTOR_SOFT_LIMIT_R);
	switch(errorcode)
	{
		case MOTOR_SOFT_LIMIT_L :
		case MOTOR_SOFT_LIMIT_R :
		{
			HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, GPIO_PIN_SET);
			// Stepper_RStop(Motor_N);
		}break;
		default :
		{
			if(DeviceStatus != DEVICE_PREPARING)
			{
				
			}
			else
			{
				HAL_GPIO_WritePin(WARNING_LED_GPIO_Port, WARNING_LED_Pin, GPIO_PIN_RESET);
			}
			// Stepper_ErrorClear(Motor_N, MOTOR_SOFT_LIMIT_L | MOTOR_SOFT_LIMIT_R);
		}break;
	}
#endif
}

void ErrorTask()
{
	if(Stepper_GetState(Motor_N) == MOTOR_ALARM)
	{
		StepperErrorHandle();
	}
	else
	{
		StepperWarningHandle();
	}
}
//用来跑会阻塞的电机任务等，先在中断中置标志位，再来下面这个函数中执行
void BlockingTask(void)
{
#if (POSITION_SENSOR_EN)
	if (isMotorReset)
	{
		isMotorReset = false;
		DeviceStatus = DEVICE_PREPARING;
		Stepper_ResetZero(Motor_N);
		DeviceStatus = DEVICE_OK;
	}
	if (isSensorCali)
	{
		isSensorCali = false;
		DeviceStatus = DEVICE_PREPARING;
		if (!Stepper_Calibration(Motor_N))
		{
			DeviceStatus |= ENCODER_CALIB_ERROR;
		}
		else
		{
			DeviceStatus = DEVICE_OK;
		}
	}
#endif
}
extern int Calibration_Angle_Read(unsigned char axis);
void TestTask(os_task* task_t)
{
	unsigned int time = HAL_GetTick();
	unsigned int time_ms = 0;

	if (time < task_t->time_ms)
	{
		time_ms = 0xFFFFFFFF + time - task_t->time_ms;
	}
	else
	{
		time_ms = time - task_t->time_ms;
	}
	if(time_ms >= task_t->interval_ms)
	{
		task_t->time_ms = time;
		if(Stepper_GetState(Motor_N) != MOTOR_STOP)
		{
			return;
		}
		switch(task_t->task_flag)
		{
			case 0:
			{
				Stepper_Run(Motor_N, DIR_FAN, OneRoundStep * MicroStep * 4);
				// Stepper_WaitUntil(Motor_N, 6000);
			}break;
			case 1:
			{
				Stepper_Run(Motor_N, DIR_ZHE, OneRoundStep * MicroStep * 4);
				// Stepper_WaitUntil(Motor_N, 6000);
			}break;
			case 2:
			{
				Stepper_Run(Motor_N, DIR_FAN, OneRoundStep * MicroStep / 200);
				// Stepper_WaitUntil(Motor_N, 6000);
			}break;
			case 3:
			{
				Stepper_Run(Motor_N, DIR_ZHE, OneRoundStep * MicroStep / 200);
				// Stepper_WaitUntil(Motor_N, 6000);
			}break;
			default: break;
		}
		// int angle = Calibration_Angle_Read(Motor_N);
		// __LOG__("Current angle : %d\n", angle);
	}
}
/****************************************      ModBus     ****************************************/
/****************************** 四个寄存器的各种操作都是在eMBPoll中执行的 ******************************/
/****************************** 所以不能执行阻塞式的代码函数会影响实时性		******************************/
//保持寄存器，读写，16bit
void gRegHoldingCBhandle(unsigned char*  yRegBuffer, unsigned short yAddress, 
                         unsigned short  yNRegs, eMBRegisterMode yMode)
{
	long temp_value1 = 0;
	long temp_value2 = 0;
	char counts = 0;
	if(yMode == MB_REG_WRITE)
	{
		switch(yAddress)
		{
			case 0x01:	//电机测试模式
			{
				
			}break;
			case 0x02:	//修改TMC2209细分
			{

			}break;
			case 0x03:	//设置加减速运动控制模式
			{
				if(yRegBuffer[1])
				{
					Stepper_SetRampMode(Motor_N, RAMP_MODE_L);
				}
				else
				{
					Stepper_SetRampMode(Motor_N, RAMP_MODE_S);
				}
			}break;
			case 0x04:	//设置加减速的步进长度
			{
				temp_value1 = (yRegBuffer[0] << 8) | yRegBuffer[1]; 
				Stepper_SetRampLength(Motor_N, temp_value1);
			}break;
			case 0x05:  //设置系统的最低转速.加减速的时候起作用.
			{
			
			}break;
			case 0x06:	//设置运行时最大转速.单位:RPM.
			{
				temp_value1 = (yRegBuffer[0] << 8) | yRegBuffer[1];
				Stepper_SetSpeed(Motor_N, (float)temp_value1);
				counts++;
				if(counts >= yNRegs) break;
			}
			case 0x07:	//设置运动方向
			{
				counts++;
				if(counts >= yNRegs) break;
			}
			case 0x08:	//设置步进距离
			case 0x09:	
			{
				counts++;
				if(counts >= yNRegs) break;
			}
			case 0x0A:
			{
				if(yRegBuffer[yNRegs * 2 - 1])
				{
					switch(yNRegs)
					{
						case 5:
						case 4:
						{
							temp_value1 =  yRegBuffer[yNRegs * 2 - 7];
							temp_value2 = (yRegBuffer[yNRegs * 2 - 6] << 8 | yRegBuffer[yNRegs * 2 - 5]) << 16 |
								            (yRegBuffer[yNRegs * 2 - 4] << 8 | yRegBuffer[yNRegs * 2 - 3]);
						}break;
						case 3:
						case 2:
						{
							temp_value1 =  usRegHoldingBuf[6];
							temp_value2 = (yRegBuffer[yNRegs * 2 - 6] << 8 | yRegBuffer[yNRegs * 2 - 5]) << 16 |
								            (yRegBuffer[yNRegs * 2 - 4] << 8 | yRegBuffer[yNRegs * 2 - 3]);
						}break;
						case 1:
						{
							temp_value1 =  usRegHoldingBuf[6];
							temp_value2 = (usRegHoldingBuf[7] << 16)|usRegHoldingBuf[8];
						}break;
						default: break;
					}
					if(Stepper_GetState(Motor_N) == MOTOR_STOP)
					{
						Stepper_Run(Motor_N, temp_value1, temp_value2);
					}	
				}	
			}break;
			case 0x0B:
			{
				if(yRegBuffer[1])
				{
					Stepper_Stop(Motor_N);
				}
			}break;
			case 0x0C:	//设置最大限位
			case 0x0D:
			{
				temp_value1 = (yRegBuffer[0] << 8 | yRegBuffer[1]) << 16 |
											(yRegBuffer[2] << 8 | yRegBuffer[3]);
				Stepper_SoftLimitMax(Motor_N, temp_value1);
			}break;
			case 0x0E:	//设置最小限位
			case 0x0F:
			{
				temp_value1 = (yRegBuffer[0] << 8 | yRegBuffer[1]) << 16 |
											(yRegBuffer[2] << 8 | yRegBuffer[3]);
				Stepper_SoftLimitMin(Motor_N, temp_value1);
			}break;
			default: break;
		}
	}
}
/****************************** 四个寄存器的各种操作都是在eMBPoll中执行的 ******************************/
/****************************** 所以不能执行阻塞式的代码函数会影响实时性		******************************/
//输入寄存器，只读，16bit
void gRegInputCBhandle(unsigned short* yRegBuffer, unsigned short yAddress, 
                       unsigned short  yNRegs)
{
	switch(yAddress)
	{
		case 0x01:
		{
			
		}break;
		case 0x02:
		{
			yRegBuffer[1] = (Stepper_GetState(Motor_N) << 8) | Stepper_ErrorStatus(Motor_N, 0xFF);
		}break;
		case 0x03:	//编码器当前值
		case 0x04:
		{
			int value = Calibration_Angle_Read(Motor_N);
			yRegBuffer[2] = (value & 0xFFFF0000) >> 16 ;
			yRegBuffer[3] = (value & 0x0000FFFF);
		}break;
		case 0x05:	//电机实时转速
		case 0x06:
		{
//			yRegBuffer[4] = ;
//			yRegBuffer[5] = ;
		}break;
		case 0x07:	//电机实时全局位置
		case 0x08:
		{
			long position = Stepper_GetPosition(Motor_N);
			yRegBuffer[6] = (position & 0xFFFF0000) >> 16;
			yRegBuffer[7] = (position & 0x0000FFFF);
		}break;
		case 0x09:	
		{
			yRegBuffer[8] = (DeviceStatus << 8);
		}break;
		case 0x0A:
		{
			yRegBuffer[9] = HAL_GPIO_ReadPin(TMC2209_EN_GPIO_Port, TMC2209_EN_Pin) << 8 |
											HAL_GPIO_ReadPin(TMC2209_DIR_GPIO_Port, TMC2209_DIR_Pin);
		}break;
		case 0x0B:
		{
			yRegBuffer[10] = (MicroStep << 8);
		}break;
		default: break;
	}
}
/****************************** 四个寄存器的各种操作都是在eMBPoll中执行的 ******************************/
/****************************** 所以不能执行阻塞式的代码函数会影响实时性		******************************/
extern void Stepper_SetPosition(unsigned char axis, long pos);
//线圈寄存器，读写，8bit
void gRegCoilsCBhandle(unsigned char* yRegBuffer, unsigned short yAddress,
                       unsigned short yNCoils, eMBRegisterMode yMode)
{
	if(yMode == MB_REG_WRITE)
	{
		switch(yAddress)
		{
			case 0x01:	//启动/关闭电机测试
			{
				if(yRegBuffer[0])
				{
					test_task.task_flag = usRegHoldingBuf[0] & 0x00FF;
				}
				else
				{
					test_task.task_flag = 255;
				}
			}break;
			case 0x02:	//启动/关闭电机待机
			{
				if(yRegBuffer[0])
				{
					TMC2209_Stdby(true);
				}
				else
				{
					TMC2209_Stdby(false);
				}
			}break;
			case 0x03:	//开启/关闭加减速运动控制
			{
				if(yRegBuffer[0])
				{
					Stepper_EnRamp(Motor_N, true);
				}
				else
				{
					Stepper_EnRamp(Motor_N, false);
				}
			}break;
			case 0x04:	//重置一次全局坐标为0
			{
				if (yRegBuffer[0])
				{
					Stepper_SetPosition(Motor_N, 0);
				}
			}break;
			case 0x05:
			{
				if (yRegBuffer[0])
				{
					if (!isMotorReset)
					{
						isMotorReset = true;
					}
				}
			}break;
			case 0x06:	//软件限位
			{
				if (yRegBuffer[0])
				{
					Stepper_SoftLimit_En(Motor_N, true);
				}
				else
				{
					Stepper_SoftLimit_En(Motor_N, false);
				}
			}break;
			case 0x07://进行一次传感器校准
			{
				if (yRegBuffer[0])
				{
					if (!isSensorCali)
					{
						isSensorCali = true;
					}
				}
			}break;
			case 0x08:
			{
				if (yRegBuffer[0])
				{
					Stepper_ErrorClear(Motor_N, MOTOR_ERROR_ALL);
				}
			}break;
			case 0x09:
			{
			
			}break;
			default: break;
		}
	}
}
/****************************** 四个寄存器的各种操作都是在eMBPoll中执行的 ******************************/
/****************************** 所以不能执行阻塞式的代码函数会影响实时性		******************************/
//离散寄存器，只读，8bit
void gRegDiscreteCBhandle(unsigned char* yRegBuffer, unsigned short yAddress, 
                          unsigned short yNDiscrete)
{
	switch(yAddress)
	{
		case 0x01:
		{

		}break;
		case 0x02:
		{
			
		}break;
		case 0x03:
		{
		
		}break;
		case 0x04:
		{
		
		}break;
		case 0x05:
		{
		
		}break;
		case 0x06:
		{
		
		}break;
		case 0x07:
		{
		
		}break;
		case 0x08:
		{
		
		}break;
		default: break;
	}
}
