/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"
#include "serialport.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
/*串口使能*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
	/* If xRXEnable enable serial receive interrupts. If xTxENable enable
	 * transmitter empty interrupts.
	 */
	if(xTxEnable == TRUE)
	{
		__HAL_UART_ENABLE_IT(&SerialPortAddress, UART_IT_TXE);
	}
	else
	{
		__HAL_UART_DISABLE_IT(&SerialPortAddress, UART_IT_TXE);
	}
}
/*串口初始化*/
BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
	/* Put a byte in the UARTs transmit buffer. This function is called
	 * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
	 * called. */
	HAL_StatusTypeDef status;
	status = HAL_UART_Transmit(&SerialPortAddress, (uint8_t *)&ucByte, 1, 50);
	if(status != HAL_OK)
	{
		return FALSE;
	}
//		USART1->TDR = ucByte;
	return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
	if(pxMBFrameCBTransmitterEmpty != 0)
	{
		pxMBFrameCBTransmitterEmpty(  );
	}
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
	if(pxMBFrameCBByteReceived != 0)
	{
		pxMBFrameCBByteReceived(  );
	}  
}

void USART1_EventProcess(void)
{	
	if(__HAL_UART_GET_IT_SOURCE(&SerialPortAddress, UART_IT_TXE) != RESET)
	{
		prvvUARTTxReadyISR();//发送中断
	}	
//		HAL_NVIC_ClearPendingIRQ(USART3_IRQn);
}

void USART1_ReceiveEventProcess(void)
{
	if(__HAL_UART_GET_FLAG(&SerialPortAddress, UART_FLAG_IDLE) != RESET)
	{
		HAL_UART_DMAStop(&SerialPortAddress);
		/*DO SOMETHINGS*/
		{
			prvvUARTRxISR();//接收中断
		}
		hal_status = HAL_UART_Receive_DMA(&SerialPortAddress, Serial1_RxBuffer, SerialPort_RxBuffer_Size);
		__HAL_UART_CLEAR_IDLEFLAG(&SerialPortAddress);	
	}
}
