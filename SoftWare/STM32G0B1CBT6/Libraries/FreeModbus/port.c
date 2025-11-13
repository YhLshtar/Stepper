#include "mb.h"
#include "mbport.h"
 
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//输入寄存器起始地址
#define REG_INPUT_START       0x0001
//输入寄存器数量
#define REG_INPUT_NREGS       11
//保持寄存器起始地址
#define REG_HOLDING_START     0x0001
//保持寄存器数量
#define REG_HOLDING_NREGS     15
//线圈起始地址
#define REG_COILS_START       0x0001
//线圈数量
#define REG_COILS_SIZE        16
//离散寄存器起始地址
#define REG_DISCRETE_START    0x0001
//离散寄存器数量
#define REG_DISCRETE_SIZE     16
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
//保持寄存器起始地址
uint16_t usRegHoldingStart = REG_HOLDING_START;
/*保持寄存器内容，读写
0x00
bit7 - 0	电机测试模式
0x01
bit7 - 0	修改TMC2209细分(预留，未实现)
0x02
bit7 - 0	设置加减速运动控制模式
0x03
bit15- 0	设置加减速的步进长度
0x04
bit15- 0	设置系统的最低转速.加减速的时候起作用.
0x05
bit15- 0	设置运行时最大转速.单位:RPM.
0x06
bit7 - 0	设置运动方向
0x07
bit32- 0	设置步进距离
0x09
bit7 - 0	开始运动
0x0A
bit7 - 0  急停
*/
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = 
{0x0000, 0x0001, 0x0000, 0x1900, 0xFFFF, 0x00A0, 0x0000, 0x0000, 0x0000, 0xFFFF,
 0xFFFF, 0x0000, 0x1900, 0x0000, 0x1900};
//输入寄存器起始地址
uint16_t usRegInputStart = REG_INPUT_START;
/*输入寄存器内容，只读
0x00
bit15- 0	固件版本
0x01 
bit15- 8	电机状态
bit7 - 0	电机错误码
0x02
bit31- 0	编码器当前值
0x04
bit31 -0	电机实时转速
0x06
bit31- 0	电机实时全局位置
0x08
bit7 - 0	整个系统的状态
0x09
bit15- 8	TMC2209EN 脚电平
bit7 - 0	TMC2209DIR脚电平
0x0A
bit15- 8	TMC2209细分
bit7 - 0
*/
uint16_t usRegInputBuf[REG_INPUT_NREGS] = 
{0x1010, 0x0000, 0x0000, 0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000,
 0x00FF};
/*输出线圈状态，读写
bit0			启动/关闭电机测试
bit1	 		启动/关闭电机待机
bit2			开启/关闭加减速运动控制
bit3			重置一次全局坐标为0
bit4			电机进行复位归零
bit5			开启/关闭软件限位
bit6			启动一次传感器校准补偿
bit7			清除所有电机错误标志位
*/
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = 
{0x20, 0xFF};
/*离散输入状态，只读

*/
uint8_t usRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = 
{0x00, 0x00};
/* USER CODE END PV */
/*上位机发来的 帧格式是:
	SlaveAddr(1 Byte) + FuncCode(1 Byte) + 
	StartAddrHiByte(1 Byte) + StartAddrLoByte(1 Byte) +
	LenAddrHiByte(1 Byte) + LenAddrLoByte(1 Byte) +
	CRCAddrHiByte(1 Byte) + CRCAddrLoByte(1 Byte)
*/
extern void  xMBUtilSetBits(UCHAR* ucByteBuf, USHORT usBitOffset, 
														UCHAR  ucNBits, UCHAR ucValue);
extern UCHAR xMBUtilGetBits(UCHAR* ucByteBuf, USHORT usBitOffset, 
														UCHAR  ucNBits);
/****************************************************************************
* 名	  称：eMBRegHoldingCB 可以读写的16bit寄存器
* 功    能：对应功能码有
*						0x06--写保持寄存器 				eMBFuncWriteHoldingRegister 
*						0x16--写多个保持寄存器 		eMBFuncWriteMultipleHoldingRegister
*						0x03--读保持寄存器 				eMBFuncReadHoldingRegister
*						0x23--读写多个保持寄存器 	eMBFuncReadWriteMultipleHoldingRegister
* 入口参数：pucRegBuffer: 		数据缓存区，用于响应主机
*						usAddress: 			寄存器地址
*						usNRegs: 				要读写的寄存器个数
*						eMode: 					功能码
* 出口参数：
****************************************************************************/
eMBErrorCode
eMBRegHoldingCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
		eMBErrorCode		eStatus = MB_ENOERR;
		int           	iRegIndex;

		unsigned short  yAddress = usAddress, yNRegs = usNRegs;
		eMBRegisterMode yMode = eMode;
		gRegHoldingCBhandle(pucRegBuffer, yAddress, yNRegs, yMode);

		if((usAddress >= REG_HOLDING_START) &&
			((usAddress + usNRegs) <= (REG_HOLDING_START + REG_HOLDING_NREGS)))
		{
				iRegIndex = (int)(usAddress - usRegHoldingStart);
				switch(eMode)
				{
						case MB_REG_READ://读 MB_REG_READ = 0
						{
								while(usNRegs > 0)
								{
										*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
										*pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
										iRegIndex++;
										usNRegs--;
								}
						}break;
						case MB_REG_WRITE://写 MB_REG_WRITE = 1
						{
								while(usNRegs > 0)
								{
										usRegHoldingBuf[iRegIndex] =  *pucRegBuffer++ << 8;
										usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
										iRegIndex++;
										usNRegs--;
								}
						}break;
				}
		}
		else//错误
		{
				eStatus = MB_ENOREG;
		}

		return eStatus;
}
/* USER CODE BEGIN PFP */
/****************************************************************************
* 名	  称：eMBRegInputCB		只读的16bit寄存器
* 功    能：对应功能码是 
*						0x04--读取输入寄存器				eMBFuncReadInputRegister
* 入口参数：pucRegBuffer: 		数据缓存区，用于响应主机
*						usAddress: 			寄存器地址
*						usNRegs: 				要读取的寄存器个数
* 出口参数：
****************************************************************************/
eMBErrorCode
eMBRegInputCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode		eStatus = MB_ENOERR;
    int           	iRegIndex;
		 
		unsigned short  yAddress = usAddress, yNRegs = usNRegs;
		gRegInputCBhandle((unsigned short*)&usRegInputBuf, yAddress, yNRegs);
	
    if((usAddress >= REG_INPUT_START) && 
			((usAddress + usNRegs) <= (REG_INPUT_START + REG_INPUT_NREGS)))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}
/****************************************************************************
* 名	  称：eMBRegCoilsCB 		可以读写的8bit寄存器
* 功    能：对应功能码有
*						0x01--读线圈 							eMBFuncReadCoils
*						0x05--写线圈 							eMBFuncWriteCoil
*						0x15--写多个线圈 					eMBFuncWriteMultipleCoils
* 入口参数：pucRegBuffer: 		数据缓存区，用于响应主机
*						usAddress: 			线圈地址
*						usNCoils: 			要读写的线圈个数
*						eMode: 					功能码
* 出口参数：
* 注	  意：如继电器 
*						0 区
****************************************************************************/
eMBErrorCode 
eMBRegCoilsCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
		eMBErrorCode		eStatus = MB_ENOERR;
		int 						iNCoils = (int)usNCoils;
		unsigned short	usBitOffset;
		
		unsigned short  yAddress = usAddress, yNRegs = usNCoils;
		eMBRegisterMode yMode = eMode;
		gRegCoilsCBhandle(pucRegBuffer, yAddress, yNRegs, yMode);
	
		if((usAddress >= REG_COILS_START) &&
			((usAddress + usNCoils) <= (REG_COILS_START + REG_COILS_SIZE)))
		{
				usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
				switch(eMode)
				{
						case MB_REG_READ://读 MB_REG_READ = 0
						{
								while( iNCoils > 0 )
								{
										*pucRegBuffer++ = 
										xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,	
																	(unsigned char)(iNCoils > 8 ? 8 : iNCoils));
										iNCoils 	  -= 8;
										usBitOffset += 8;
								}
						}break;
						case MB_REG_WRITE://写 MB_REG_WRITE = 1
						{
								while( iNCoils > 0 )
								{
										xMBUtilSetBits(ucRegCoilsBuf, usBitOffset,	
																	(unsigned char)(iNCoils > 8 ? 8 : iNCoils),	
																	 *pucRegBuffer++);
										iNCoils     -= 8;
										usBitOffset += 8;
								}
						}break;
				}
		}
		else
		{
				eStatus = MB_ENOREG;
		}
		
		return eStatus;
}
/****************************************************************************
* 名	  称：eMBRegDiscreteCB	只读的8bit寄存器
* 功    能：对应功能码有
*						0x02--读离散寄存器 				eMBFuncReadDiscreteInputs
* 入口参数：pucRegBuffer: 		数据缓存区，用于响应主机
*						usAddress: 			寄存器地址
*						usNDiscrete: 		要读取的寄存器个数
* 出口参数：	
* 注	  意：1 区
****************************************************************************/
eMBErrorCode
eMBRegDiscreteCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    eMBErrorCode		eStatus    = MB_ENOERR;
    short         	iNDiscrete = (short)usNDiscrete;
    unsigned short  usBitOffset;

		unsigned short  yAddress = usAddress, yNRegs = usNDiscrete;
		gRegDiscreteCBhandle((unsigned char*)&usRegDiscreteBuf, yAddress, yNRegs);
	
    if((usAddress >= REG_DISCRETE_START ) && 
			((usAddress + usNDiscrete) <= (REG_DISCRETE_START + REG_DISCRETE_SIZE)))
    {
        usBitOffset = (unsigned short)(usAddress - REG_DISCRETE_START);
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
            xMBUtilGetBits(usRegDiscreteBuf, usBitOffset,
													(unsigned char)(iNDiscrete > 8 ? 8 : iNDiscrete));
            iNDiscrete  -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
		
    return eStatus;
}
/*For User Define Function , cmd range 0x41-0x48*/
eMBErrorCode
eMBUser1CB1(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
		unsigned char   receiveData;
		unsigned short 	Address = usAddress;
		eMBErrorCode		eStatus    = MB_ENOERR;
	
		if(Address == 0)
		{
				unsigned short dataLength  = usNDiscrete;
			  for(unsigned short i = 0; i < dataLength; i++)
				{
						receiveData = *(pucRegBuffer + i);
				}
		}
		else
		{
				eStatus = MB_ENOREG;
		}
	
		return eStatus;
}
/*For User Define Function ,cmd range 0x64-0x6E*/
eMBErrorCode
eMBUser2CB1(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
		unsigned char   receiveData;
		unsigned short 	Address = usAddress;
		eMBErrorCode		eStatus    = MB_ENOERR;
	
		if(Address == 0)
		{
				unsigned short dataLength  = usNDiscrete;
			  for(unsigned short i = 0; i < dataLength; i++)
				{
						receiveData = *(pucRegBuffer + i);
				}
		}
		else
		{
				eStatus = MB_ENOREG;
		}
	
		return eStatus;
}
//保持寄存器，读写，16bit
__WEAK void gRegHoldingCBhandle(unsigned char* yRegBuffer, unsigned short yAddress, 
																unsigned short yNRegs, eMBRegisterMode yMode)
{

}
//输入寄存器，只读，16bit
__WEAK void gRegInputCBhandle(unsigned short* yRegBuffer, unsigned short yAddress, 
														  unsigned short  yNRegs)
{

}
//线圈寄存器，读写，8bit
__WEAK void gRegCoilsCBhandle(unsigned char* yRegBuffer, unsigned short yAddress, 
															unsigned short yNCoils, eMBRegisterMode yMode)
{

}
//离散寄存器，只读，8bit
__WEAK void gRegDiscreteCBhandle(unsigned char* yRegBuffer, unsigned short yAddress, 
																 unsigned short yNDiscrete)
{

}

/* USER CODE END PFP */

 