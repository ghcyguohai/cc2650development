

#include "Ring_Buf.h"
/*
typedef struct 
{
 uint16_t*  write_index;
 uint16_t*  read_index;
 uint16_t   max_size_mask;
 uint8_t*   buff_addr;
}RING_BUF_DEF_STRUCT;
*/

RING_BUF_DEF_STRUCT  USB_Recieve_Ringbuf;
uint8_t              USB_Rx_Buf[USB_RX_BUFSIZE];




void  RingBuf_Init(RING_BUF_DEF_STRUCT* ring_buf,uint8_t* src_buf,uint16_t src_buf_maxsize)
{
ring_buf->write_index=0;
ring_buf->read_index=0;
ring_buf->buff_addr=src_buf;
ring_buf->max_size_mask=(src_buf_maxsize-1);
}

void  RingBuf_Read(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len,uint8_t* dst_buf)
{
  uint16_t count=0;

  if(src_ringbuf->buff_addr!=NULL)  
  {
    for (count = 0; count<len;count++)
    {
      dst_buf[count]=src_ringbuf->buff_addr[src_ringbuf->read_index++];
			src_ringbuf->read_index=(src_ringbuf->read_index&src_ringbuf->max_size_mask);
    }
  }
 else
 {
 	 // add error process
 }
}

void      RingBuf_Peek (RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len ,uint8_t* dst_buf)
{
   uint16_t count=0;
  if(src_ringbuf->buff_addr!=NULL)  
  {
    for (count = 0; count<len;count++)
    {
      dst_buf[count]=src_ringbuf->buff_addr[src_ringbuf->peek_index++];
     src_ringbuf->peek_index=(src_ringbuf->peek_index&src_ringbuf->max_size_mask);
    }
  }
 else
 {

   // add error process
 }

}

void  RingBuf_Peeked_Flush(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t len)
{
  src_ringbuf->read_index=src_ringbuf->peek_index;
}

void      RingBuf_Peeked_Reset (RING_BUF_DEF_STRUCT* src_ringbuf)
{
  src_ringbuf->peek_index=src_ringbuf->read_index;
}

void  RingBuf_Write(RING_BUF_DEF_STRUCT* des_ringbuf,uint8_t* src, uint16_t len)
{
  uint16_t count=0;
 
 if(RingBuf_UnusedCount(des_ringbuf)<len)
 {
    count=len;
 }      
 if(des_ringbuf->buff_addr!=NULL) 
 { 
   for(count=0;count<len;count++)
   {
    des_ringbuf->buff_addr[des_ringbuf->write_index++]=src[count];
		des_ringbuf->write_index=(des_ringbuf->write_index&des_ringbuf->max_size_mask);
   }
 } 
 else
 {
 	 // add error  process
 }
}

uint16_t  RingBuf_UnusedCount(RING_BUF_DEF_STRUCT* src_ringbuf)
{
    return ((src_ringbuf->max_size_mask+1)-RingBuf_Count(src_ringbuf));
}

uint16_t  RingBuf_Count(RING_BUF_DEF_STRUCT* src_ringbuf)
{
 uint16_t count=0;
 count=(src_ringbuf->write_index - src_ringbuf->read_index)&(src_ringbuf->max_size_mask);
return count;
}
void  RingBuf_Flush(RING_BUF_DEF_STRUCT* src_ringbuf)
{
  src_ringbuf->write_index=0;
  src_ringbuf->read_index=0;
	src_ringbuf->peek_index=0;
}

uint16_t  RingBuf_Peeked_Counter(RING_BUF_DEF_STRUCT* src_ringbuf)
{
  uint16_t count;
  count=(src_ringbuf->peek_index-src_ringbuf->read_index)&(src_ringbuf->max_size_mask);
  return count;
}

uint16_t  RingBuf_PeekCount(RING_BUF_DEF_STRUCT* src_ringbuf)
{
 uint16_t count=0;
 count=(src_ringbuf->write_index - src_ringbuf->peek_index)&(src_ringbuf->max_size_mask);
	return count;
}

void RingBuf_Peek_Pos_Change(RING_BUF_DEF_STRUCT* src_ringbuf,uint16_t offset)
{
   src_ringbuf->peek_index=(src_ringbuf->read_index+offset)&(src_ringbuf->max_size_mask);
}
