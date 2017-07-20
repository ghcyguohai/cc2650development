#ifndef  UART3_TASK_H

#define  UART3_TASK_H


#include "Ring_Buf.h"


void   AppTask_Uart3_Create(void);

static void AppTask_Uart3(void *pvParameters);

       void  AppTask_Uart3_Init(void);
static void  Uart3_Config(void);
static void  Uart3_Rx_Dma_Config(void);
       void  Uart3_DMA_isr(void);
static void Uart3_Dma_RX_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf);
#endif 

