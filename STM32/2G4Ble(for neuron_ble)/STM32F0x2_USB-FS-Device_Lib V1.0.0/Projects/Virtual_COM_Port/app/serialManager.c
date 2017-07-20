/* Standard includes. */
#include <stdio.h>
#include <string.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* hardware includes */
#include "hw_config.h"
#include "serialManager.h"

#include <stdio.h>
#include <stdarg.h>

//#define   PRINGF_DEBUG_UART1
//#define   PRINGF_DEBUG_UART2
//#define   PRINGF_DEBUG_UART3
//#define   PRINTF_DEBUG_USB

void uart1_printf(char *fmt, ...);
void uart2_printf(char *fmt, ...);

void USB_Printf(char* fmt,...)
{
#ifdef PRINTF_DEBUG_USB
    
va_list ap;
char string[128];
va_start(ap, fmt);
vsprintf(string, fmt, ap);
sendString_ViaUSB(string);
va_end(ap);
    
#endif     
}

void sendString_ViaUSB(char *str)
{
    uint8_t temp[128]={0};
    uint8_t send_len=0;
    
    while(*str)
    {
     temp[send_len++]=*str++;
    }
    USB_CDC_Send(temp,send_len);
}
void sendString_uart3(char *str)
{
  while(*str)
  {
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    USART_SendData(USART3, *str++);
  }
}

void sendString_uart2(char *str)
{
  while(*str)
  {
   	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, *str++);
  }
}

void sendString_uart1(char *str)
{
  while(*str)
  {
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, *str++);
	
  }
}

void uart1_printf(char *fmt, ...)
{
	#ifdef PRINGF_DEBUG_UART1
		va_list ap;
		char string[32];
		va_start(ap, fmt);
		vsprintf(string, fmt, ap);
		sendString_uart1(string);
		va_end(ap);
	#endif 
}

void uart2_printf(char *fmt, ...)
{
#ifdef PRINGF_DEBUG_UART2
	
  va_list ap;
  char string[32];
  va_start(ap, fmt);
  vsprintf(string, fmt, ap);
  sendString_uart2(string);
  va_end(ap);
	
#endif 
}



void uart2write(char *str, int len)
{
  while(len--)
  {
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, *str++);
  }
}

void uart_3write(char *str, int len)
{
  while(len--)
  {
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    USART_SendData(USART3, *str++);
  }
}
void uart3_printf(char *fmt, ...)
{
	#ifdef   PRINGF_DEBUG_UART2
  va_list ap;
  char string[32];
  va_start(ap, fmt);
  vsprintf(string, fmt, ap);
  sendString_uart3(string);
  va_end(ap);
	#endif 
}

int fputc(int ch, FILE *f)
{
  /* Loop until the end of transmission */
 	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  USART_SendData(USART1, (uint8_t) ch);
  return ch;
}


 void  Uart_Send(USART_TypeDef* UART,uint8_t* src ,uint16_t len)
{
	uint16_t cnt=0;
	    for(cnt=0;cnt<len;cnt++)
	    {
            while (USART_GetFlagStatus(UART, USART_FLAG_TC) == RESET);
            USART_SendData(UART,src[cnt]);
	    }
}



