/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Add By 2024.12 
 * User Define Handle Function
 */
/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbframe.h"
#include "mbproto.h"
#include "mbconfig.h"

/* ----------------------- Defines ------------------------------------------*/


/* ----------------------- Static functions ---------------------------------*/
eMBException    prveMBError2Exception( eMBErrorCode eErrorCode );

/* ----------------------- Start implementation -----------------------------*/

#if MB_FUNC_USER_DEFINE_ENABLED > 0

eMBException    
eMBFuncUser1Define1( UCHAR * pucFrame, USHORT * usLen )
{
		eMBException    eStatus = MB_EX_NONE;
		eMBErrorCode    eRegStatus;
		/*lose head and tail*/
		eRegStatus = eMBUser1CB1( ++pucFrame, 0, (*usLen - 1) );
	
		return eStatus;
}

#endif

#if MB_FUNC_USER_DEFINE_ENABLED > 0

eMBException    
eMBFuncUser2Define1( UCHAR * pucFrame, USHORT * usLen )
{
		eMBException    eStatus = MB_EX_NONE;
		eMBErrorCode    eRegStatus;
		/*lose head and tail*/
		eRegStatus = eMBUser2CB1( ++pucFrame, 0, (*usLen - 1));
	
		return eStatus;
}

#endif