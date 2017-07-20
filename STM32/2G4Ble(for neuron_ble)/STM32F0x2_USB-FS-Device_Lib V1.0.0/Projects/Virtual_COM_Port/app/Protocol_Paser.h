


#ifndef  PROTOCOL_H

#define  PROTOCOL_H

#include "stdint.h"
#include  "Ring_Buf.h"
typedef  __packed  struct 
{
  uint16_t  sof;
  uint8_t   protocol_ver;
 uint16_t   module_serial_num;
 uint16_t   opcode :15;
 uint16_t   ack:1;
 uint16_t   data_len;
 uint8_t    crc8;
}FRAME_HEADER;


typedef struct 
{
  RING_BUF_DEF_STRUCT* ring_buf;
  uint8_t              step;
  void*                parser_cache;
  uint16_t             parser_cache_index;

}PARSER_HANDLE;





#endif 