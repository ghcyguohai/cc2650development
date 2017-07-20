#ifndef  UART2_TASK_H

#define  UART2_TASK_H





void   AppTask_Uart2_Create(void);

static void AppTask_Uart2(void *pvParameters);

       void  AppTask_Uart2_Init(void);
static void  Uart2_Config(void);
static void  Uart2_Rx_Dma_Config(void);
       void  Uart2_RingBuf_Rx_Flush(void);
#endif 
