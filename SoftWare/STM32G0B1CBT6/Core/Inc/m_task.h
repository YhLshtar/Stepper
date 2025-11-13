#ifndef __M_TASK_H
#define __M_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Stepper_control.h"
#include "mb.h"

#define DEVICE_OK             0B00000000
#define DEVICE_PREPARING      0B00000001
#define ENCODER_INIT_ERROR    0B00000010
#define ENCODER_CALIB_ERROR   0B00000100

#define FDCAN_COMMUNICATION   1
#define UART_COMMUNICATION    0

typedef struct OS_TCB
{
  volatile unsigned char	task_flag;
  volatile unsigned char  task_id;
  unsigned int	  				interval_ms;
  unsigned int            time_ms;
}os_task;

extern unsigned char DeviceStatus;
extern unsigned char CommunicationMode;

void m_OSKernelInit(void);
void m_OSKernelStart(void);

#ifdef __cplusplus
}
#endif

#endif /* __M_TASK_H */
