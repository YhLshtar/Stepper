#include "m_fdcan.h"
#include "fdcan.h"

#define   TJA1051_SILENT_ON()   HAL_GPIO_WritePin(TJA1051T_S_GPIO_Port, TJA1051T_S_Pin, GPIO_PIN_SET)
#define   TJA1051_SILENT_OFF()  HAL_GPIO_WritePin(TJA1051T_S_GPIO_Port, TJA1051T_S_Pin, GPIO_PIN_RESET)

FDCAN_TxHeaderTypeDef TxHeader_Standard;
FDCAN_TxHeaderTypeDef TxHeader_Extended;
FDCAN_RxHeaderTypeDef RxHeader;
HAL_StatusTypeDef     fdcan_status;

unsigned char MessageMarker = 0;
unsigned char RxData[FDCAN_RxBuffer_Size] = {0};
unsigned char TxData[64] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC};

char BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

void FDCAN_Filter_Init(void)
{
  FDCAN_FilterTypeDef   sFilterConfig;
/*##-1 Configure the FDCAN filters ########################################*/
/* Configure standard ID reception filter to Rx FIFO 0
 * 配置标准帧(2^11)的过滤器, 接受特定ID
 */
  sFilterConfig.IdType        = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex   = 0;
  sFilterConfig.FilterType    = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig  = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1     = 0x01;
  sFilterConfig.FilterID2     = 0x02;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
/* Configure extended ID reception filter to Rx FIFO 1
 * 配置扩展帧(2^29)的过滤器
 */
  sFilterConfig.IdType        = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex   = 0;
  sFilterConfig.FilterType    = FDCAN_FILTER_RANGE_NO_EIDM;
  sFilterConfig.FilterConfig  = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1     = 0x01;
  sFilterConfig.FilterID2     = 0x02;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
/* Configure global filter: Filter all remote frames and non-matching frames with STD and EXT ID
 * 拒绝ID所有不匹配的帧，当然也可以选择接收ID不匹配的帧进入FIFO0或者FIFO1，用来设置白名单和黑名单ID
*/
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                     FDCAN_REJECT,
                     FDCAN_REJECT,
                     FDCAN_FILTER_REMOTE,
                     FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN_Transmit_Standard_Init(unsigned int bit_rate_switch)
{
  TxHeader_Standard.IdType        = FDCAN_STANDARD_ID;
  TxHeader_Standard.TxFrameType   = FDCAN_DATA_FRAME;
  TxHeader_Standard.BitRateSwitch = bit_rate_switch;
  TxHeader_Standard.FDFormat             = FDCAN_FD_CAN;
  TxHeader_Extended.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
  TxHeader_Standard.TxEventFifoControl   = FDCAN_STORE_TX_EVENTS;
}

void FDCAN_Transmit_Extended_Init(unsigned int bit_rate_switch)
{
  TxHeader_Extended.IdType        = FDCAN_EXTENDED_ID;
  TxHeader_Extended.TxFrameType   = FDCAN_DATA_FRAME;
  TxHeader_Extended.BitRateSwitch = bit_rate_switch;
  TxHeader_Extended.FDFormat             = FDCAN_FD_CAN;
  TxHeader_Extended.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
  TxHeader_Extended.TxEventFifoControl   = FDCAN_STORE_TX_EVENTS;
}

void FDCAN_Init(void)
{
  FDCAN_Filter_Init();
  FDCAN_Transmit_Standard_Init(FDCAN_BRS_OFF);
/* Activate Rx FIFO 0 new message notification */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
/* Configure and enable Tx Delay Compensation, required for BRS mode.
   TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
   TdcFilter default recommended value: 0 */
  if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
/*##-2 Start FDCAN controller (continuous listening CAN bus) ##############*/
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
/*关闭静音模式, 收发器坏了的情况下可以打开, 防止坏的收发器影响整个CAN网络*/
  TJA1051_SILENT_OFF();
  while (1)
  {
    FDCAN_Standard_Transmit(0x01, TxData, FDCAN_DLC_BYTES_12);
    HAL_Delay(1000);
    TxData[0]  = 0xFF;
    TxData[13] = 0xFF;
    FDCAN_Standard_Transmit(0x01, TxData, FDCAN_DLC_BYTES_16);
    HAL_Delay(1000);
  }
}

char FDCAN_Standard_Transmit(unsigned int id, unsigned char* data, unsigned int length)
{
  unsigned char t_err = (hfdcan1.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
  unsigned char r_err = (hfdcan1.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
/* Add message to Tx FIFO */
  TxHeader_Standard.Identifier = id;
  TxHeader_Standard.DataLength = length;
  if ((t_err > 127) || (r_err > 127))
  {
    TxHeader_Extended.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  }
  TxHeader_Standard.MessageMarker = MessageMarker++;
  fdcan_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Standard, data);
  if (fdcan_status != HAL_OK)
  {
    return 0;
  }
// /* Wait transmissions complete */
//   while(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3)
//   {
//
//   }
// /*##-4 Receive messages ###################################################*/
// /* Check one message is received in Rx FIFO 0 */
//   if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 1)
//   {
//     Error_Handler();
//   }
// /* Retrieve message from Rx FIFO 0 */
//   if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//   {
//     Error_Handler();
//   }
// /* Compare payload to expected data */
//   if (BufferCmp8b(TxData, RxData, 12) != 0)
//   {
//     Error_Handler();
//   }
  return 1;
}

char FDCAN_Extended_Transmit(unsigned int id, unsigned char* data, unsigned int length)
{
  unsigned char t_err = (hfdcan1.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
  unsigned char r_err = (hfdcan1.Instance->ECR & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
/* Add second message to Tx FIFO */
  TxHeader_Extended.Identifier = id;
  TxHeader_Extended.DataLength = length;
  if ((t_err > 127) || (r_err > 127))
  {
    TxHeader_Extended.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
  }
  TxHeader_Extended.MessageMarker = MessageMarker++;
  fdcan_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_Extended, data);
  if (fdcan_status != HAL_OK)
  {
    return 0;
  }
/* Wait transmissions complete */
  // while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 3)
  // {
  //
  // }
  // /*##-4 Receive messages ###################################################*/
  // /* Check one message is received in Rx FIFO 0 */
  // if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 1)
  // {
  //   Error_Handler();
  // }
  // /* Retrieve message from Rx FIFO 0 */
  // if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // /* Compare payload to expected data */
  // if (BufferCmp8b(TxData, RxData, 12) != 0)
  // {
  //   Error_Handler();
  // }
  return 1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    // 获得接收到的数据头和数据
    fdcan_status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
    if (fdcan_status == HAL_OK)
    {
      // RxHeader.IdType
      //HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);	// 再次使能FIFO0接收中断
    }
  }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

}

char BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}