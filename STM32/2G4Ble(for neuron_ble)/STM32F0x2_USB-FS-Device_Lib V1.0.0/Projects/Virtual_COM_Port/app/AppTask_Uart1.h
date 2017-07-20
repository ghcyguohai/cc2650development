#ifndef  UART1_TASK_H

#define  UART1_TASK_H
#include "stdint.h"
#include "stm32f0xx.h"


void   AppTask_Uart1_Create(void);

static void  AppTask_Uart1(void *pvParameters);
       void  AppTask_Uart1_Init(void);
static void  Uart1_Config(void);
static void	 Uart1_Rx_Dma_Config(void);
       void USB_User_Recieve_ISR(uint8_t* pbuf,uint16_t len);
#endif 

