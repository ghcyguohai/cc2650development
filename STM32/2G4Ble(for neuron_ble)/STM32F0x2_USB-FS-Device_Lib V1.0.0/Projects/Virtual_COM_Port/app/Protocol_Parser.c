
//#include "Protocol_Paser.h"
//#include "Ring_Buf.h"

////typedef  __packed  struct 
////{
////  uint16_t  sof;
////  uint8_t   protocol_ver;
//// uint16_t   module_serial_num;
//// uint16_t   opcode :15;
//// uint16_t   ack:1;
//// uint16_t   data_len;
//// uint8_t    crc8;
////}FRAME_HEADER;




//typedef struct 
//{
//  RING_BUF_DEF_STRUCT* ring_buf;
//  uint8_t              step;
//  void*                parser_cache;
//  uint16_t             parser_cache_index;
//}PARSER_HANDLE;




//uint16_t  Pack_Data(uint8_t* src,uint16_t len)
//{

//FRAME_HEADER*  p_frameheader=(FRAME_HEADER*) src;

//p_frameheader->sof=0x55aa;
//Attach_Crc8(p_frameheader,sizeof(FRAME_HEADER)-1)
//Attach_Crc16(p_frameheader,len);
//return (len+2);//(Crc16=2bytes)
//}

//void  Cmd_Packet_Send(uint8_t protocol_ver,uint16_t module_serial_num,uint16_t opcode,uint16_t ack,uint8_t* data_src, uint16_t data_len)
//{
//uint8_t  cache[512];
//uint16_t send_len=0;
//FRAME_HEADER* p_frameheader=(FRAME_HEADER*) cache;

//p_frameheader->protocol_ver=protocol_ver;
//p_frameheader->module_serial_num=module_serial_num;
//p_frameheader->opcode=opcode;
//p_frameheader->ack=ack;
//p_frameheader->data_len=data_len;

//memcpy(&cache[sizeof(FRAME_HEADER)],data_src,len);
//send_len=Pack_Data(cache,data_len+sizeof(FRAME_HEADER));

///*               attach low level send interface            */ 

//}

//void Cmd_Unpack(PARSER_HANDLE* parser_handle)
//{
//   uint8_t temp=0;
//   uint8_t header_index;

// while(RingBuf_PeekCount(parser_handle->ring_buf))
// {

//    RingBuf_Peek(parser_handle->RingBuf_Read,1,&temp);

//   switch(parser_handle->step)
//   {
//     case 0:   // srf
//     
//      if(temp=0x55)
//      {
//      	parser_handle->parser_cache_index[parser_handle->parser_cache_index++];
//      	parser_handle->step=1;
//      }

//     break;
//      

//     case 1:   // AA

//      if(temp=0xAA)
//      {
//      	parser_handle->parser_cache_index[parser_handle->parser_cache_index++];
//      	parser_handle->step=2;
//      	header_len=sizeof(FRAME_HEADER)-2;

//        for(header_len=0;header_len<(sizeof(FRAME_HEADER)-2);header_len++)
//        {
//           RingBuf_Peek(parser_handle->RingBuf_Read,1,&temp);
//           parser_handle->parser_cache[parser_cache_index++]=temp;
//        }

//        if(Check_Crc8(parser_handle->parser_cache_index,sizeof(FRAME_HEADER))==0)   // CRC OK
//        {
//           parser_handle                                     // add func code 
//        }
//        else
//        {
//        	
//        }

//      }
//     break;


//   }



// }



//	
//}

