#include "TMC2209.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

/*----------TMC UART CONFIGURATION----------*/
#define	USE_UART_CONFIG_EN		1	
#if	 	USE_UART_CONFIG_EN
#define UART_MIRCO_STEP_EN		0			//细分UART配置开启，不生效，固定还是MS1和MS2的值，挠头~~
#define	GLOBAL_CONFIG_WR			0x00
#define UART_EACH_CNT_WR			0x02
#define	IOINPUT_STATUS_WR			0x06
#define	CHOPPER_CONTROL_WR		0x6C

volatile char UartRecFlag = 0;
unsigned char UartReceive[12] = {0};
unsigned char g_data[4] 			= {0};
#endif

bool TMC2209_UARTWrite(char reg, unsigned char* dataW);
bool TMC2209_UARTRead(char reg, unsigned char* dataR);
void TMC2209_MicroStep_Set(MicroStepType type);

void TMC2209_Init(void)
{
	// TMC2209_En(false);				//关闭内部电机驱动器
	// TMC2209_Stdby(false);			//取消待机模式
	TMC2209_MicroStep_Set(MicroStep);	//设置细分
	TMC2209_MotorDir(MotorREV);
	TMC2209_Spread(true);			//关掉斩波模式
	TMC2209_Stdby(false);			//取消待机模式
	TMC2209_En(true);					//使能内部电机驱动器
}

void TMC2209_En(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(TMC2209_EN_GPIO_Port, TMC2209_EN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(TMC2209_EN_GPIO_Port, TMC2209_EN_Pin, GPIO_PIN_SET);
	}
}

void TMC2209_Stdby(bool enable)
{
	if(enable)
	{
		HAL_GPIO_WritePin(TMC2209_STDBY_GPIO_Port, TMC2209_STDBY_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(TMC2209_STDBY_GPIO_Port, TMC2209_STDBY_Pin, GPIO_PIN_RESET);
	}
}

void TMC2209_Spread(bool enable)
{
	if(enable)	//StealthChop	低速超静音模式
	{
		HAL_GPIO_WritePin(TMC2209_SPR_GPIO_Port, TMC2209_SPR_Pin, GPIO_PIN_RESET);
	}
	else		//SpreadCycle	高速抑制共振模式
	{
		HAL_GPIO_WritePin(TMC2209_SPR_GPIO_Port, TMC2209_SPR_Pin, GPIO_PIN_SET);
	}
}

void TMC2209_MotorDir(unsigned char dir)
{
	HAL_GPIO_WritePin(TMC2209_DIR_GPIO_Port, TMC2209_DIR_Pin, dir);
}

void TMC2209_MicroStep_Set(MicroStepType type)
{
#if	UART_MIRCO_STEP_EN
	HAL_GPIO_WritePin(TMC2209_MS1_GPIO_Port, TMC2209_MS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TMC2209_MS2_GPIO_Port, TMC2209_MS2_Pin, GPIO_PIN_RESET);		
	HAL_Delay(10);	
	if(TMC2209_UARTRead(CHOPPER_CONTROL_WR, &g_data[0]))
	{
		g_data[0] = 0x15;
		g_data[3] = g_data[3] & 0x07;//
		switch(type)
		{
			case MicroStep_Full:
			{
				g_data[3] = g_data[3] | (8 << 4);	
			}break;
			case MicroStep_2:
			{
				g_data[3] = g_data[3] | (7 << 4);	
			}break;
			case MicroStep_4:
			{
				g_data[3] = g_data[3] | (6 << 4);	
			}break;
			case MicroStep_8:
			{
				g_data[3] = g_data[3] | (5 << 4);	
			}break;
			case MicroStep_16:
			{
				g_data[3] = g_data[3] | (4 << 4);	
			}break;
			case MicroStep_32:
			{
				g_data[3] = g_data[3] | (3 << 4);	
			}break;
			case MicroStep_64:
			{
				g_data[3] = g_data[3] | (2 << 4);	
			}break;
			case MicroStep_128:
			{
				g_data[3] = g_data[3] | (1 << 4);	
			}break;
			case MicroStep_256:
			{
				g_data[3] = g_data[3] | (1 << 3);	
			}break;
			default:
			{
				
			}break;
		}
		TMC2209_UARTWrite(CHOPPER_CONTROL_WR, &g_data[0]);
		TMC2209_UARTRead(CHOPPER_CONTROL_WR, &g_data[0]);
	}
	else
	{
		return ;
	}
	if(TMC2209_UARTRead(GLOBAL_CONFIG_WR, &g_data[0]))
	{
		g_data[3] = g_data[3] & 0x3B;//傻逼，第3位给0写进去是1，给1写进去是0
		g_data[3] = g_data[3] | 0xC0;//开启内部细分 禁止PDN_UART的硬件触发低电流模式
		TMC2209_UARTWrite(GLOBAL_CONFIG_WR, &g_data[0]);
	}
#else
	switch(type)
	{
		case MicroStep_8:
		{
			HAL_GPIO_WritePin(TMC2209_MS1_GPIO_Port, TMC2209_MS1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TMC2209_MS2_GPIO_Port, TMC2209_MS2_Pin, GPIO_PIN_RESET);
		}break;
		case MicroStep_16:
		{
			HAL_GPIO_WritePin(TMC2209_MS1_GPIO_Port, TMC2209_MS1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TMC2209_MS2_GPIO_Port, TMC2209_MS2_Pin, GPIO_PIN_SET);
		}break;
		case MicroStep_32:
		{
			HAL_GPIO_WritePin(TMC2209_MS1_GPIO_Port, TMC2209_MS1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(TMC2209_MS2_GPIO_Port, TMC2209_MS2_Pin, GPIO_PIN_RESET);
		}break;
		case MicroStep_64:
		{
			HAL_GPIO_WritePin(TMC2209_MS1_GPIO_Port, TMC2209_MS1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(TMC2209_MS2_GPIO_Port, TMC2209_MS2_Pin, GPIO_PIN_SET);
		}break;
		default:
		{
			
		}break;
	}
#endif
}
/*
转速和PWM的频率有关，按照1.8°转距角的步进电机为例。
主控发送一个脉冲，电机转动1.8°/ 细分N (根据实际驱动芯片细分设置计算)，
转速 = ((频率hz * (1.8 / N )) / 360°)(秒/圈) 
*/
void TMC2209_StepFreq_Set(unsigned int hz)
{
//  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, hz);
//	__HAL_TIM_SET_PRESCALER(&htim3, hz);
	__HAL_TIM_SET_AUTORELOAD(&htim3, hz);
}

void TMC2209_StepFreq_OnlyLevel(unsigned char level)
{
	if(level)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 65535);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	}
}

void TMC2209_StepFreq_OnlyHigh()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 65535);
}

void TMC2209_StepFreq_OnlyLow()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
}
#if	USE_UART_CONFIG_EN
unsigned char CRC8(unsigned char* data, unsigned char dataLen) 
{ 
	// CRC located in last byte of message 
	unsigned char crcData = 0; 
	unsigned char currentByte; 

	for (int i = 0; i < dataLen; i++) 
	{      
		currentByte = data[i];                
		// Execute for all bytes of a message 
		// Retrieve a byte to be sent from Array 
		for (int j = 0; j < 8; j++) 
		{ // update CRC based result of XOR operation 
			if ((crcData >> 7) ^ (currentByte & 0x01))   
			{ 
				crcData = (crcData << 1) ^ 0x07; 
			} 
			else 
			{ 
				crcData = (crcData << 1); 
			} 
			currentByte = currentByte >> 1; 
		} // for CRC bit 
	} // for message byte 
	
	return crcData;
} 

bool TMC2209_UARTRead(char reg, unsigned char* dataR)
{
	bool res = false;
	unsigned char datar[4] = {0};
	unsigned char rev_crc = 0;
	
	datar[0] = 0x05;
	datar[1] = 0x00;
	datar[2] = reg;
	datar[3] = CRC8(datar, 3);
	
	HAL_UART_Receive_IT(&huart2, &UartReceive[0], sizeof(UartReceive)/sizeof(UartReceive[0]));
	if(HAL_UART_Transmit(&huart2, datar, sizeof(datar)/ sizeof(datar[0]), 100) == HAL_OK)
	{
		while(!UartRecFlag)
		{

		}
		UartRecFlag = 0;
		rev_crc = CRC8(&UartReceive[4], 7);
		if(rev_crc != UartReceive[11])
		{
			res = false;
		}
		else
		{
			dataR[0] = UartReceive[7];
			dataR[1] = UartReceive[8];
			dataR[2] = UartReceive[9];
			dataR[3] = UartReceive[10];
			res = true;
		}
	}
	memset(UartReceive, 0x00, sizeof(UartReceive)/sizeof(UartReceive[0]));
	
	return res;
}

bool TMC2209_UARTWrite(char reg, unsigned char* dataW)
{
	bool					res = false;
	unsigned char datar[4] = {0};
	unsigned char dataw[8] = {0};
	
	dataw[0] = 0x05;
	dataw[1] = 0x00;
	dataw[2] = reg | 0x80;
	dataw[3] = dataW[0];
	dataw[4] = dataW[1];
	dataw[5] = dataW[2];
	dataw[6] = dataW[3];
	dataw[7] = CRC8(dataw, 7);
	
	HAL_UART_Receive_IT(&huart2, &UartReceive[0], sizeof(dataw)/sizeof(dataw[0]));
	if(HAL_UART_Transmit(&huart2, dataw, sizeof(dataw)/ sizeof(dataw[0]), 100) == HAL_OK)
	{
		while(!UartRecFlag)
		{

		}
		UartRecFlag = 0;
		res = true;
	}
	memset(UartReceive, 0x00, sizeof(UartReceive)/sizeof(UartReceive[0]));
	
	return res;
}

void UART2_RxCompleteHandle()
{
	if(UartReceive[0] == 0x05)
	{
		UartRecFlag = 1;
	}
	else
	{
		
	}
}

#endif
	
	