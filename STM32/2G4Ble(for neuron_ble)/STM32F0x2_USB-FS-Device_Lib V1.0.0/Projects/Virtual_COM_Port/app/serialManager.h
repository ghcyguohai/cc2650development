#ifndef SERIALMANAGER_H
#define SERIALMANAGER_H

#include "stm32f0xx.h"

void serialMngInit(void);
void uart2write(char *str, int len);
void parseDbgMsg(char *cmd);
void USB_Printf(char* fmt,...);
void sendString_ViaUSB(char *str);
void uart2_printf(char *fmt, ...);
void uart1_printf(char *fmt, ...);
void uart3_printf(char *fmt, ...);
void initUart1Peri(void);
void initUart2Peri(void);
void  Uart_Send(USART_TypeDef* UART,uint8_t* src ,uint16_t len);
#endif
