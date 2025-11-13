#include "string.h"
#include "serialport.h"

//ALIGN_32BYTES (unsigned char Serial1_RxBuffer[SerialPort_RxBuffer_Size]) = {0};
unsigned char Serial1_RxBuffer[SerialPort_RxBuffer_Size] = {0};

HAL_StatusTypeDef hal_status = 0;

void SerialPort_Init(void)
{
//  __HAL_UART_DISABLE(&SerialPortAddress);
	hal_status = HAL_UART_Receive_DMA(&SerialPortAddress, Serial1_RxBuffer, SerialPort_RxBuffer_Size);
  __HAL_UART_CLEAR_IDLEFLAG(&SerialPortAddress);
	__HAL_UART_ENABLE_IT(&SerialPortAddress, UART_IT_IDLE);
//  __HAL_UART_ENABLE(&SerialPortAddress);
//	HAL_UARTEx_ReceiveToIdle_DMA(&SerialPortAddress, Serial1_RxBuffer, SerialPort_RxBuffer_Size)
}

void UART1_RxCompleteHandle()
{
/*串口DMA非循环模式时，要判断缓冲区数据是否溢出*/
/*TODO*/
	memset(Serial1_RxBuffer, 0, SerialPort_RxBuffer_Size);
}

__WEAK void USART1_EventProcess(void)
{

}

__WEAK void USART1_ReceiveEventProcess(void)
{

}

int fputc(int ch, FILE *f)
{
	while(SerialPortAddress.gState != HAL_UART_STATE_READY)
	{
		
	}
/* 循环阻塞发送需要消耗大量CPU资源“搬运”数据，浪费CPU时间，不过无需使用串口中断 */
	HAL_UART_Transmit(&SerialPortAddress, (uint8_t *)&ch, 1, 50);
	
	return ch;
}

#if (SerialPort_DMA_Send == 1)
HAL_StatusTypeDef LOG_Debug(const char* format, ...)
{
	HAL_StatusTypeDef status;
	int Legth = 0;
	
	va_list arg;
	va_start(arg, format);
	Legth = vsnprintf((char*)Serial1_TxBuffer,sizeof(Serial1_TxBuffer),(char*)format, arg);
	va_end(arg);
#if (SerialPort_DMA_Send_Block == 1)
	while(SerialPortAddress.gState != HAL_UART_STATE_READY)
	{
		
	}	
#endif
	status = HAL_UART_Transmit_DMA(&SerialPortAddress,(uint8_t *)Serial1_TxBuffer, Legth);
	
	return status;
}
#endif
