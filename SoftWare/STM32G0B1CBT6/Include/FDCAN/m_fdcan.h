#ifndef __M_FDCAN_H
#define __M_FDCAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#define FDCAN_RxBuffer_Size				256

void FDCAN_Init(void);
void FDCAN_Transmit_Standard_Init(unsigned int bit_rate_switch);
void FDCAN_Transmit_Extended_Init(unsigned int bit_rate_switch);
char FDCAN_Standard_Transmit(unsigned int id, unsigned char* data, unsigned int length);
char FDCAN_Extended_Transmit(unsigned int id, unsigned char* data, unsigned int length);

#ifdef __cplusplus
}
#endif

#endif /* __M_FDCAN_H */