#include "m_timers.h"
#include "tim.h"
#include "mb.h"
#include "Stepper_control.h"

#define GET_CPU_ClkFreq()       (SystemCoreClock)
#define SysClockFreq            (SystemCoreClock)

#define  DWT_CR      						*(__IO uint32_t *)0xE0001000
#define  DWT_CYCCNT  						*(__IO uint32_t *)0xE0001004
#define  DEM_CR      						*(__IO uint32_t *)0xE000EDFC

#define  DEM_CR_TRCENA          (1 << 24)
#define  DWT_CR_CYCCNTENA       (1 <<  0)
 
void DWT_Init(void)
{
	/* 使能DWT外设 */
	DEM_CR |= (unsigned int)DEM_CR_TRCENA;
	/* DWT CYCCNT寄存器计数清0 */
	DWT_CYCCNT = (uint32_t)0u;
	/* 使能Cortex-M DWT CYCCNT寄存器 */
	DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

unsigned int DWTCount_Read(void)
{        
  return (unsigned int)DWT_CYCCNT;
}

void Timers_Start(unsigned char index)
{
	switch(index)
	{
		case TIMER1:
		{

		}break;
		case TIMER2:
		{
			
		}break;
		case TIMER3:
		{

		}break;
		case TIMER4:
		{

		}break;
		case TIMER5:
		{
			
		}break;
		case TIMER6:
		{
			HAL_TIM_Base_Start_IT(&htim6);
		}break;
		case TIMER7:
		{
			HAL_TIM_Base_Start_IT(&htim7);
		}break;
		case TIMER8:
		{
			
		}break;
		case TIMER9:
		{
			
		}break;
		case TIMER10:
		{
			
		}break;
		case TIMER11:
		{
			
		}break;
		case TIMER12:
		{
			
		}break;
		case TIMER13:
		{
			
		}break;
		case TIMER14:
		{

		}break;
		case TIMER15:
		{
			
		}break;
		case TIMER16:
		{

		}break;
		case TIMER17:
		{

		}break;
		default : break;
	}
}

#if (isTimerDelay == TIME_TIMER)
/*
* 函数名：void us_timer_delay(uint16_t t)
* 输入参数：t-延时时间 us
* 输出参数：无
* 返回值：无
* 函数作用：定时器实现的延时函数，延时时间为 t us，为了缩短时间，
*	函数体使用寄存器操作，用户可对照手册查看每个寄存器每一位的意义
*/
void timer_delay_us(unsigned short t)
{
	__HAL_TIM_SET_AUTORELOAD(&DelayTimer, t);
	__HAL_TIM_SET_COUNTER(&DelayTimer, 0);
	HAL_TIM_Base_Start(&DelayTimer);
	while(__HAL_TIM_GET_COUNTER(&DelayTimer) < t)
	{
		
	}
	HAL_TIM_Base_Stop(&DelayTimer);
//	__HAL_TIM_SET_AUTORELOAD(&DelayTimer, t - 1);//定时器响应时间为period*定时器频率
//	HAL_TIM_Base_Start(&DelayTimer);//start the timer
//	//通过轮询的方式等待定时器的更新事件
//	//当定时器溢出并计数器更新时，TIM_FLAG_UPDATE标志会被置位。
//	while(__HAL_TIM_GET_FLAG(&DelayTimer,TIM_FLAG_UPDATE)==RESET);
//	__HAL_TIM_CLEAR_FLAG(&DelayTimer,TIM_FLAG_UPDATE);//清楚更新标志位
//		HAL_TIM_Base_Stop(&DelayTimer);//Stop the timer
}
#elif (isTimerDelay == TIME_DWT)
void timer_delay_us(unsigned short t)
{
  unsigned int ticks;
  unsigned int told, tnow, tcnt = 0;

  DWT_Init();
  
  ticks = t * (GET_CPU_ClkFreq() / 1000000);  /* 需要的节拍数 */    
  tcnt  = 0;
  told  = (unsigned int)DWTCount_Read();      /* 刚进入时的计数器值 */


  while(1)
  {
    tnow = (unsigned int)DWTCount_Read();  
    if(tnow != told)
    { 
			/* 32位计数器是递增计数器 */  
      if(tnow > told)
      {
        tcnt += tnow - told;  
      }
      /* 重新装载 */
      else 
      {
        tcnt += UINT32_MAX - told + tnow; 
      } 
      told = tnow;
      /*时间超过/等于要延迟的时间,则退出 */
      if(tcnt >= ticks)break;
    }  
  }
}
#else
/*
* 函数名：void us_timer_delay(uint16_t t)
* 输入参数：t-延时时间 us
* 输出参数：无
* 返回值：无
* 函数作用：延时粗略实现的延时函数，延时时间为 t us。根据不同主控的主频需要计算和调整
*/
void timer_delay_us(unsigned short t)
{
	uint16_t counter = 0;
	while(t--) 
	{
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
		__NOP();__NOP();
	} 
}

#endif

void timer_delay_ms(unsigned short t)
{
	while(t--) 
	{
		timer_delay_us(1000);
	} 
}

void timer_delay_s(unsigned short t)
{
	while(t--) 
	{
		timer_delay_ms(1000);
	} 
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6)
	{
		(void)eMBPoll();
	}
	// else if (htim->Instance == TIM7)
	// {
	//
	// }
	else if (htim->Instance == TIM14)
	{
#if (POSITION_SENSOR_EN)
		Stepper_Position_Scan(Motor_N);
#endif
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	//	if(htim->Instance == TIM3)
	//	{
	Stepper_Scan(Motor_N);
	//	}
}
