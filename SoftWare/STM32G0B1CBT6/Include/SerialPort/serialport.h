#ifndef __SERIALPORT_H
#define __SERIALPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "usart.h"
#include "stm32g0xx_hal.h"
/* 	

*/
#define SerialPort_DMA_Send             0U
#define SerialPort_DMA_Send_Block       1U

#define SerialPort_TxBuffer_Size      	128
#define SerialPort_RxBuffer_Size				256

#define SerialPortID										USART1
#define SerialPortAddress								huart1

extern HAL_StatusTypeDef hal_status;
extern unsigned char 		 Serial1_RxBuffer[SerialPort_RxBuffer_Size];

void SerialPort_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERIALPORT_H */