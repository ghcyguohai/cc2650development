#ifndef  OLD_PROTOCOL_H

#define   OLD_PROTOCOL_H
#include "stdint.h"
#include "Ring_Buf.h"
#include "stm32f0xx.h"
#include "AppTask_Start.h"

typedef struct 
{
  RING_BUF_DEF_STRUCT* ring_buf;
  PASSTHROUGH_HANDLE*  passthrough_handle;
  uint8_t              step;
  uint8_t*             parser_cache;
  uint16_t             parser_cache_index;
  uint16_t             parser_cache_maxsize;

}PARSER_HANDLE;

void    Cmd_Pack_Send(USART_TypeDef* Uart,uint8_t* src ,uint8_t len);
void    Cmd_Unpack(PARSER_HANDLE* parser_handle);
uint8_t Check_Sum(uint8_t* src,uint16_t len);
void    Uart_PassThrough(PARSER_HANDLE* parser_handle,uint16_t passthrough_len );
#endif 

