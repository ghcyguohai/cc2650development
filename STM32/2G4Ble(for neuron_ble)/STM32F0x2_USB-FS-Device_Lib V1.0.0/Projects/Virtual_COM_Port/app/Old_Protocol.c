#include <string.h>
#include "Old_Protocol.h"
#include "serialManager.h"
#include "AppTask_BroadcomBle.h"
#include "hw_config.h"
//#define UART_LOOP_TEST  1



void Cmd_Pack_Send(USART_TypeDef* Uart,uint8_t* src ,uint8_t len)
{
 uint8_t temp[128];
 uint8_t checksum=0;
 uint8_t i=0;
 memset(temp,0x00,128);
 temp[0]=0xFA; // add sof 
 memcpy(&temp[1],src,len);
 
for (i = 0; i < len; i++)
{
  checksum+=temp[1+i];
}
temp[len+1]=checksum;  
temp[len+2]=0xFB;

Uart_Send(Uart,temp,len+3);
}

void Cmd_Unpack(PARSER_HANDLE* parser_handle)
{
   uint8_t temp=0;

 while(RingBuf_PeekCount(parser_handle->ring_buf))
 {
   switch(parser_handle->step)
   {
     case 0:   // sof
       RingBuf_Peek(parser_handle->ring_buf,1,&temp);
          if(temp==0xFA)  //  unused data SOF useful data  EOF 
          {
            // if the find the sof, we should pass the peeked datas, 1: sof 
            Uart_PassThrough(parser_handle,RingBuf_Peeked_Counter(parser_handle->ring_buf)-1);                              // send  pass_through data
            parser_handle->parser_cache_index=0; // rest trans pos 
            parser_handle->parser_cache[parser_handle->parser_cache_index++]=temp;
            parser_handle->step=1;
          }
          else
           {
            Uart_PassThrough(parser_handle,RingBuf_Peeked_Counter(parser_handle->ring_buf));
           }
     break;
    
     case 1:   
			
       RingBuf_Peek(parser_handle->ring_buf,1,&temp);
		 
      if((temp==0xFB)&&((parser_handle->parser_cache_maxsize>parser_handle->parser_cache_index)))  // END OF, over protect 
      {			
				  parser_handle->parser_cache[parser_handle->parser_cache_index++]=temp;
					if(Check_Sum(parser_handle->parser_cache,parser_handle->parser_cache_index)==0)  // check sum
					{
						parser_handle->step=0;	//  add code 
						RingBuf_Peeked_Flush(parser_handle->ring_buf,parser_handle->parser_cache_index); // reconfig the read_index
						
						if(parser_handle->ring_buf->uart==USART1)  // just allow the request come frome the uart1. when the module is master
							{
								USART_Cmd(USART2,DISABLE); // stop the uart2 trans	
                                BleRecived_Slave_Broadcast_cmd_Ack(&parser_handle->parser_cache[1]);
							}
						
						if(parser_handle->ring_buf->uart==USART2)   // whem the module is slave ,the request comes from UART2
							{
								USART_Cmd(USART1,DISABLE); // stop the uart1 trans
                                BleRecieved_Mater_Broadcast_cmd(&parser_handle->parser_cache[1]);
							}	
                            
					  BleCmd_Request_Notifiacation(); // if recieved cmd; notification the AppTask_WIFI. the wifi Task will handle the data of the packet
					}
          else // check sume failed, the data need to be passthrough to the associated port
          {
				    parser_handle->parser_cache_index=0;
				    RingBuf_Peeked_Reset(parser_handle->ring_buf);	  //  reconfig the peed index 
                    RingBuf_Peek_Pos_Change(parser_handle->ring_buf,1); // the pos is used to  find the next sof
				    Uart_PassThrough(parser_handle,RingBuf_Peeked_Counter(parser_handle->ring_buf));	// pass the unused data
                    parser_handle->step=0;   // go back to step 0
          }
      }
      else if(parser_handle->parser_cache_index==parser_handle->parser_cache_maxsize) // if  cache buffer is full ,but no EOF is found. 
			 {
				  parser_handle->parser_cache_index=0;
				  parser_handle->step=0; //go back to the fist step to find the next sof 
				  RingBuf_Peeked_Reset(parser_handle->ring_buf);	  // reconfig the peed index 
			      RingBuf_Peek_Pos_Change(parser_handle->ring_buf,1);  //the pos is used to  find the next sof
				  Uart_PassThrough(parser_handle,RingBuf_Peeked_Counter(parser_handle->ring_buf));	// pass the unused data
			 }
     else   // no EOF is found and the cache buffer is not full
			 {
				parser_handle->parser_cache[parser_handle->parser_cache_index++]=temp;
			 }
     break;
   }
 }
}

uint8_t Check_Sum(uint8_t* src,uint16_t len)
{
  uint8_t temp=0;
  uint16_t i=0;

 for (i = 0; i < len-3; i++)  // sof+endof + checsum=3;
 {
 	  temp=src[i+1]+temp;
 }

 if(temp==src[len-2])
 	return 0;
 else
 	return 1;
}



void Uart_PassThrough(PARSER_HANDLE* parser_handle,uint16_t passthrough_len )
{
	uint8_t temp=0;
    uint8_t usb_sendbuf[62]={0};
  Check_Connect_Mode(parser_handle->passthrough_handle);
	
#if  UART_LOOP_TEST
  	parser_handle->passthrough_handle->passthrough_dir=LOOP_BACK;
#endif 	
if (parser_handle->passthrough_handle->passthrough_dir==LOOP_BACK)
	{
		while(passthrough_len)
		{
		 passthrough_len--;
         RingBuf_Read(parser_handle->ring_buf,1,&temp);
	 	 if(parser_handle->ring_buf->uart!=NULL)
         {
           Uart_Send(parser_handle->ring_buf->uart,&temp,1);
         }
		}
	}
 else if( parser_handle->passthrough_handle->passthrough_dir==UART1_TO_UART3)
 {
    if(parser_handle->ring_buf->uart==USART1)
	 {
		 while(passthrough_len)
		 {
		    passthrough_len--;
			RingBuf_Read(parser_handle->ring_buf,1,&temp);
            Uart_Send(USART3,&temp,1);
		 }
	 }
	 else if(parser_handle->ring_buf->uart==USART3)
	 {
		 while(passthrough_len)
		 {
		  passthrough_len--;
		  RingBuf_Read(parser_handle->ring_buf,1,&temp);
          Uart_Send(USART1,&temp,1);
		 }
	 }
 }
 else if (parser_handle->passthrough_handle->passthrough_dir==UART2_TO_UART3)
 {
	 if(parser_handle->ring_buf->uart==USART2)
	 {
		 while(passthrough_len)
		 {
              passthrough_len--;
              RingBuf_Read(parser_handle->ring_buf,1,&temp);
              Uart_Send(USART3,&temp,1); 
		 }
	 }
	 else if(parser_handle->ring_buf->uart==USART3)
	 { 
		 while(passthrough_len)
		 {
              passthrough_len--;
              RingBuf_Read(parser_handle->ring_buf,1,&temp);
              Uart_Send(USART2,&temp,1);
		 }
	 }
 }
 else if(parser_handle->passthrough_handle->passthrough_dir==UART3_TO_UART1andUart2) 
 {
	  if(parser_handle->ring_buf->uart==USART3)
		{
		 while(passthrough_len)   //  clear the uused data that come from the uart2.
		 {
              passthrough_len--;
              RingBuf_Read(parser_handle->ring_buf,1,&temp);
           // Uart_Send(USART1,&temp,1); 
              Uart_Send(USART2,&temp,1);
		 }
	 }
     else if(parser_handle->ring_buf->uart==USART2)
     {
          while(passthrough_len)   //  clear the uused data that come from the uart2.
		 {
              passthrough_len--;
              RingBuf_Read(parser_handle->ring_buf,1,&temp);
           // Uart_Send(USART1,&temp,1); 
              Uart_Send(USART3,&temp,1);
		 }
     
     }
 }
 else if(parser_handle->passthrough_handle->passthrough_dir==UART2_TO_USB)
 {
   if(parser_handle->ring_buf->uart==NULL) // from usb 
   {
         while(passthrough_len)   //  clear the uused data that come from the uart2.
			 {
				passthrough_len--;
				RingBuf_Read(parser_handle->ring_buf,1,&temp);
				Uart_Send(USART2,&temp,1);
			 }
   }
   
   if(parser_handle->ring_buf->uart==USART2)   // uart2 for bluetootch
   {
         while(passthrough_len)   // 
			 {
                if(passthrough_len/64>0) 
                {
                   passthrough_len-=64; 
                   RingBuf_Read(parser_handle->ring_buf,64,usb_sendbuf);
                   USB_CDC_Send(usb_sendbuf,64);
                }
				else if((passthrough_len%64)>0) 
                {
                   RingBuf_Read(parser_handle->ring_buf,passthrough_len,usb_sendbuf);
                   USB_CDC_Send(usb_sendbuf,passthrough_len);
                   passthrough_len=0;
                }		
			 }
   }
 
 }
 else if (parser_handle->passthrough_handle->passthrough_dir==UART3_TO_USB) 
 {
   if(parser_handle->ring_buf->uart==NULL) // from usb 
   {
         while(passthrough_len)   //  clear the uused data that come from the uart2.
			 {
				passthrough_len--;
				RingBuf_Read(parser_handle->ring_buf,1,&temp);
				Uart_Send(USART3,&temp,1);
			 }
   }
   
   if(parser_handle->ring_buf->uart==USART3)
   {
         while(passthrough_len)   // 
			 {
		
                 
                if(passthrough_len/64>0) 
                {
                   passthrough_len-=64; 
                   RingBuf_Read(parser_handle->ring_buf,64,usb_sendbuf);
                   USB_CDC_Send(usb_sendbuf,64);
                }
				else if((passthrough_len%64)>0) 
                {
                   RingBuf_Read(parser_handle->ring_buf,passthrough_len,usb_sendbuf);
                   USB_CDC_Send(usb_sendbuf,passthrough_len);
                   passthrough_len=0;
                }		
			 }
   }
 }
 else if(parser_handle->passthrough_handle->passthrough_dir==STOP_UART3_PASSTHROUGH)
 {
    if(parser_handle->ring_buf->uart!=USART3)  // if the data are not from uart3 ,empty the input buffer
    {
      while(passthrough_len)   //  clear the uused data that come from the uart2.
            {
				passthrough_len--;
				RingBuf_Read(parser_handle->ring_buf,1,&temp);
				Uart_Send(USART3,&temp,1);
			}
    }
 
 }
 else 
 {
    while(passthrough_len)   //  clear the uused data that come from the uart2.
	{
		passthrough_len--;
		RingBuf_Read(parser_handle->ring_buf,1,&temp);
	}
 }
}
