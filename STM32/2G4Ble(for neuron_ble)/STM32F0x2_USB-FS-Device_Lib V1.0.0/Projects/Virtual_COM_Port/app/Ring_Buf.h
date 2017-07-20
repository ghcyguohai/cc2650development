

#ifndef  RING_BUF_H

#define  RING_BUF_H
#include "stdint.h"
#include "stdio.h"


#define  USB_RX_BUFSIZE  512

typedef struct 
{
 void*      uart;
 volatile   uint16_t  write_index;
 volatile   uint16_t  read_index;
 volatile   uint16_t  peek_index;
 uint16_t   max_size_mask;
 uint8_t*   buff_addr;
}RING_BUF_DEF_STRUCT;


void      RingBuf_Init(RING_BUF_DEF_STRUCT* ring_buf,uint8_t* src_buf,uint16_t src_buf_maxsize);
void      RingBuf_Read(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len,uint8_t* dst_buf);
void      RingBuf_Peek (RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len ,uint8_t* dst_buf);
void      RingBuf_Peek_Pos_Change(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t offset);
uint16_t  RingBuf_Peeked_Counter(RING_BUF_DEF_STRUCT* src_ringbuf);
void      RingBuf_Peeked_Flush(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len);
void      RingBuf_Peeked_Reset (RING_BUF_DEF_STRUCT* src_ringbuf);
void      RingBuf_Write(RING_BUF_DEF_STRUCT* des_ringbuf,uint8_t* src, uint16_t len);
void      RingBuf_Flush(RING_BUF_DEF_STRUCT* src_ringbuf);
uint16_t  RingBuf_Count(RING_BUF_DEF_STRUCT* src_ringbuf);
uint16_t  RingBuf_UnusedCount(RING_BUF_DEF_STRUCT* src_ringbuf);
uint16_t  RingBuf_PeekCount(RING_BUF_DEF_STRUCT* src_ringbuf);

#endif 
