
#include "FreeRTOS.h"
#include "Task.h"
#include "Broadcom_Ble.h"
#include "stm32f0xx.h"
#include  "string.h"
#include "stdio.h"
#include "Old_Protocol.h"
#include "AppTask_Uart3.h"
#include "serialManager.h"


#define FLASH_PAGE_ADDR(page_num)  (0x08000000+0x400*page_num)  // the range is 0-127,one page is 1k
#define BLE_INFO_ADDR               FLASH_PAGE_ADDR(127)


extern PARSER_HANDLE   Uart3_Protocol_Unpack_Handle;
extern PASSTHROUGH_HANDLE        Passthrough_Handle;


BLE_CFGINFO Ble_Cfg_Mac={0,0,{0},{0}}; 

static PARSER_HANDLE* Com_Rx_Handle=&Uart3_Protocol_Unpack_Handle;  


void    Ble_Info_Init(void)
{  
    Ble_Info_Read(&Ble_Cfg_Mac);  // read self mac info  
    if((Ble_Cfg_Mac.ble_mode!=0x00)&&(Ble_Cfg_Mac.ble_mode!=0x01))  // default the save info
        {   
           if( Ble_ATCmd_ReadMac(Ble_Cfg_Mac.ble_selfmac,12)==0)  // read successfully
               { 
                 USB_Printf("ble cfg default set \r\n");  
                 Ble_Cfg_Mac.ble_mode=1;  // 1:master, 0 slave 
                 Ble_Info_Write(&Ble_Cfg_Mac);
               }
           else
               {
                 //while(1);
               }
        }  
        
    if(Ble_ATCmdPassthrough_Send()==0)
    {
        USB_Printf("start up set mac successfully \r\n");  
    }
    
    RingBuf_Flush(Uart3_Protocol_Unpack_Handle.ring_buf);  
}

void    Ble_Info_Read(BLE_CFGINFO* dst)
{
    uint8_t temp[16]={0};
    uint8_t read4_len=1;
  
    if((sizeof(BLE_CFGINFO)%4)>0)
        read4_len=(sizeof(BLE_CFGINFO)/4)+1;
    else
    {
        read4_len=1;
    }
    
    memcpy(temp,(uint8_t*)BLE_INFO_ADDR,read4_len*4);
    memcpy(dst,temp,sizeof(BLE_CFGINFO));
}

void    Ble_Info_Write(BLE_CFGINFO* src)
{
    uint8_t write_len_4bytes;
    uint8_t remain_bytes;
    uint8_t i=0;	
    uint32_t* u32_ptr=(uint32_t*)src;	
        
    write_len_4bytes=sizeof(BLE_CFGINFO)/4;
    remain_bytes=	sizeof(BLE_CFGINFO)%4;

    taskDISABLE_INTERRUPTS();
        
    FLASH_Unlock();
    FLASH_ErasePage(BLE_INFO_ADDR);
    FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPERR);	
        
    for(i=0;i<write_len_4bytes;i++)
    {
     FLASH_ProgramWord(BLE_INFO_ADDR+i*4,u32_ptr[i]);	
    }
    if(remain_bytes!=0)
    {
     FLASH_ProgramWord(BLE_INFO_ADDR+write_len_4bytes*4,u32_ptr[write_len_4bytes]);
    }	
    FLASH_Lock();	

    taskENABLE_INTERRUPTS();
}

void Ble_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;  

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // ble reset pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOB, GPIO_Pin_4);  
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;   // this io is used to read ble connectton level
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//¸¡¿ÕÊäÈë
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
    
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  GPIO_SetBits(GPIOC, GPIO_Pin_13);    
}


static void Ble_CmdPin_Set(uint8_t mode)
{
  if(mode==1)
  {      
      GPIO_SetBits(GPIOC, GPIO_Pin_13); // 0: cmd mode. 1: exit cmd mode 
  }
  else 
  {
     GPIO_ResetBits(GPIOC, GPIO_Pin_13);  
  }
}

void    Ble_ATCmdAutoLink(void)
{
    char str[128];
    uint8_t  send_len=0;
    memset(str,0x00,128);
    send_len=sprintf(str,"AT+LINK=0");
    str[send_len]=0x0d;
    str[send_len+1]=0x0a;
    RingBuf_Flush(Com_Rx_Handle->ring_buf);	
    Uart_Send(Com_Rx_Handle->ring_buf->uart,(uint8_t*)str,send_len+2);
    vTaskDelay( 50/portTICK_RATE_MS ); 	
}

void  Ble_ConnectStatus_Monitor(void)
{
  if(GPIO_ReadInputDataBit( GPIOB,  GPIO_Pin_14)==0x01)
	 {
	   Passthrough_Handle.uart_online_starus.uart3_online_singal=1;
	 }
	 else
	 {
	   Passthrough_Handle.uart_online_starus.uart3_online_singal=0;
	 }
//   if(GPIO_ReadInputDataBit( GPIOB,  GPIO_Pin_6)==0x01)  
//   {
//       Passthrough_Handle.uart_online_starus.usb_online_singal=0;  
//   } 
//   else
//    {
//       Passthrough_Handle.uart_online_starus.usb_online_singal=1;  
//    }
}

uint8_t  Ble_ATCmdPassthrough_Send(void)
{
    char str[128];
    uint8_t  cmd_send_cnt=0;
    uint8_t  send_len=0;
    TRANS_DIR  temp_trans_dir;

    memset(str,0x00,128);
    temp_trans_dir=Passthrough_Handle.passthrough_dir;
    Passthrough_Handle.passthrough_dir=STOP_UART3_PASSTHROUGH;  // save current trans_dir
    Ble_CmdPin_Set(0); // first  low cmd 
    do
    {
        cmd_send_cnt++;
        vTaskDelay( 50/portTICK_RATE_MS );
    }while(Ble_ATCmdDisconnecct()&&(cmd_send_cnt<10));
    
    if(cmd_send_cnt>=10)
    {
         Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
         Ble_CmdPin_Set(1); // release set pin      
      return 1;  // disconnect fail
    }
    cmd_send_cnt=0;  // clear the cnt 
    if(Ble_Cfg_Mac.ble_mode==1)
    {    
           send_len=sprintf(str,"AT+LINK=1,%c%c%c%c%c%c%c%c%c%c%c%c",
                    Ble_Cfg_Mac.ble_targetmac[0],Ble_Cfg_Mac.ble_targetmac[1],Ble_Cfg_Mac.ble_targetmac[2],
                    Ble_Cfg_Mac.ble_targetmac[3],Ble_Cfg_Mac.ble_targetmac[4],Ble_Cfg_Mac.ble_targetmac[5],
                    Ble_Cfg_Mac.ble_targetmac[6],Ble_Cfg_Mac.ble_targetmac[7],Ble_Cfg_Mac.ble_targetmac[8],
                    Ble_Cfg_Mac.ble_targetmac[9],Ble_Cfg_Mac.ble_targetmac[10],Ble_Cfg_Mac.ble_targetmac[11]);
    }
    else if(Ble_Cfg_Mac.ble_mode==0)  // AT_LINK=2;, set Slave connect address
    {
           send_len=sprintf(str,"AT+LINK=2,%c%c%c%c%c%c%c%c%c%c%c%c",
                    Ble_Cfg_Mac.ble_targetmac[0],Ble_Cfg_Mac.ble_targetmac[1],Ble_Cfg_Mac.ble_targetmac[2],
                    Ble_Cfg_Mac.ble_targetmac[3],Ble_Cfg_Mac.ble_targetmac[4],Ble_Cfg_Mac.ble_targetmac[5],
                    Ble_Cfg_Mac.ble_targetmac[6],Ble_Cfg_Mac.ble_targetmac[7],Ble_Cfg_Mac.ble_targetmac[8],
                    Ble_Cfg_Mac.ble_targetmac[9],Ble_Cfg_Mac.ble_targetmac[10],Ble_Cfg_Mac.ble_targetmac[11]);
    }

    str[send_len]=0x0d;
    str[send_len+1]=0x0a;
    do
    {
        cmd_send_cnt++;
        RingBuf_Flush(Com_Rx_Handle->ring_buf);	
        Uart_Send(Com_Rx_Handle->ring_buf->uart,(uint8_t*)str,send_len+2);
        vTaskDelay( 50/portTICK_RATE_MS ); 	
    }while(Ble_ATCmdPassthrough_Res_Check(Com_Rx_Handle->ring_buf)&&(cmd_send_cnt<10));
    
     Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
     Ble_CmdPin_Set(1); // release set pin
    
     if(cmd_send_cnt<10)
        return 0;
     else
        return 1;
}

void    Ble_ATCmd_Tick(void)
{
 static  uint8_t  send_counter=0;
    
    if(send_counter<10)
    {
       send_counter++; 
    }    
    else
    {
       send_counter=0;
       if(Com_Rx_Handle->passthrough_handle->uart_online_starus.uart3_online_singal==0)
           {
               if( Ble_ATCmdPassthrough_Send()!=0)
               {
                    USB_Printf("send set mac failed \r\n");  
               }
           }
    }
}
uint8_t Ble_ATCmd_ReadMac(uint8_t* p_buf,uint8_t len)
{
    char str[128];
    uint8_t  send_len=0;
    uint8_t cmd_send_cnt=0;
    TRANS_DIR temp_trans_dir;
    memset(str,0x00,128);
    temp_trans_dir=Passthrough_Handle.passthrough_dir;
    Passthrough_Handle.passthrough_dir=STOP_UART3_PASSTHROUGH;  // save current trans_dir
    
    Ble_CmdPin_Set(0); // first  low cmd 
    do
    {
        cmd_send_cnt++;
        vTaskDelay( 50/portTICK_RATE_MS );
    }while(Ble_ATCmdDisconnecct()&&(cmd_send_cnt<10));
    
    if(cmd_send_cnt>=10)
    {
      Ble_CmdPin_Set(1); // release cmd pin 
      Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
      return 1;  // disconnect fail
    }
    cmd_send_cnt=0;
    
    send_len=sprintf(str,"AT+LEMAC?"),
    str[send_len]=0x0d;
    str[send_len+1]=0x0a;
    
    do
    {
        cmd_send_cnt++;
        RingBuf_Flush(Com_Rx_Handle->ring_buf);	
        Uart_Send(Com_Rx_Handle->ring_buf->uart,(uint8_t*)str,send_len+2);
        vTaskDelay( 50/portTICK_RATE_MS ); 	
    }while(Ble_ATCmd_ReadMac_Res_Check(Com_Rx_Handle->ring_buf)&&(cmd_send_cnt<10));
    
     Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
     Ble_CmdPin_Set(1); // release cmd pin 
     if(cmd_send_cnt<10)
     {
        return 0;
     }
     else
     {
        return 1;
     }
}

uint8_t Ble_ATCmd_ReadTargetMac(uint8_t* p_buf,uint8_t len)
{
  char str[128];
    uint8_t  send_len=0;
    uint8_t cmd_send_cnt=0;
    TRANS_DIR temp_trans_dir;
    memset(str,0x00,128);
    temp_trans_dir=Passthrough_Handle.passthrough_dir;
    Passthrough_Handle.passthrough_dir=STOP_UART3_PASSTHROUGH;  // save current trans_dir
    
    Ble_CmdPin_Set(0); // first  low cmd 
    do
    {
        cmd_send_cnt++;
        vTaskDelay( 50/portTICK_RATE_MS );
    }while(Ble_ATCmdDisconnecct()&&(cmd_send_cnt<10));
    
    if(cmd_send_cnt>=10)
    {
      return 1;  // disconnect fail
    }
    
    send_len=sprintf(str,"AT+LINK?"),
    str[send_len]=0x0d;
    str[send_len+1]=0x0a;
    RingBuf_Flush(Com_Rx_Handle->ring_buf);	
    Uart_Send(Com_Rx_Handle->ring_buf->uart,(uint8_t*)str,send_len+2);
    vTaskDelay( 50/portTICK_RATE_MS ); 	
    
	 if(Ble_ATCmd_ReadMac_Res_Check(Com_Rx_Handle->ring_buf)==0)
	 {
        Ble_CmdPin_Set(1); // release cmd pin 
        Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
        return 0;
	 }	 
	 else
	 {
        Ble_CmdPin_Set(1); // release cmd pin 
        Passthrough_Handle.passthrough_dir=temp_trans_dir;  // restore trans dir 
        return 1;
	 }	    
}


static uint8_t Ble_ATCmdDisconnecct(void)
{
    char str[128];
    uint8_t  send_len=0;
    memset(str,0x00,128);

    send_len=sprintf(str,"AT+DISC=1");
    str[send_len]=0x0d;
    str[send_len+1]=0x0a;
    RingBuf_Flush(Com_Rx_Handle->ring_buf);	
    Uart_Send(Com_Rx_Handle->ring_buf->uart,(uint8_t*)str,send_len+2);
     vTaskDelay( 50/portTICK_RATE_MS ); 	
	 if(Ble_ATCmdDisconnecct_Res_Check(Com_Rx_Handle->ring_buf)==0)
	 {
		return 0;
	 }	 
	 else
	 {
        return 1;
	 }	   
}

static uint8_t Ble_ATCmdPassthrough_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf)
{
    uint8_t rx_buf[32];
    uint8_t rx_cnt;

 memset(rx_buf,0x00,32);
 rx_cnt=RingBuf_Count(Com_Rx_Handle->ring_buf);

 if(rx_cnt>32)
 	rx_cnt=32;

 RingBuf_Read(Com_Rx_Handle->ring_buf,rx_cnt,rx_buf);
	
 if(StringFind((char*)rx_buf,"OK")>=0)
 {
  return 0;  // ok
 }
 else
 	return 1;   

}

static uint8_t Ble_ATCmdDisconnecct_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf)
{
      uint8_t rx_buf[32];
      uint8_t rx_cnt;

     memset(rx_buf,0x00,32);
     rx_cnt=RingBuf_Count(Com_Rx_Handle->ring_buf);

     if(rx_cnt>32)
        rx_cnt=32;

     RingBuf_Read(Com_Rx_Handle->ring_buf,rx_cnt,rx_buf);
        
     if(StringFind((char*)rx_buf,"OK")>=0)
     {
      return 0;  // ok
     }
     else
        return 1;   
}

static uint8_t Ble_ATCmd_ReadMac_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf)
{
     uint8_t rx_buf[32];
     uint8_t rx_cnt;
     

     memset(rx_buf,0x00,32);
     rx_cnt=RingBuf_Count(Com_Rx_Handle->ring_buf);

     if(rx_cnt>32)
        rx_cnt=32;

     RingBuf_Read(Com_Rx_Handle->ring_buf,rx_cnt,rx_buf);
     if(rx_cnt==23)  // return datalen 23 byte
     {
       if(StringFind((char*)rx_buf,"+LEMAC")>=0)
         {
           memcpy(Ble_Cfg_Mac.ble_selfmac,&rx_buf[9],12);
           return 0;         
         }  
       else
           {
              return 1; 
           }
     }
     else
         return 1;

}

static int StringFind(const char *pSrc, const char *pDst)  
{  
    int i, j;  
    for (i=0; pSrc[i]!='\0'; i++)  
    {  
        if(pSrc[i]!=pDst[0])  
            continue;         
        j = 0;  
        while(pDst[j]!='\0' && pSrc[i+j]!='\0')  
        {  
            j++;  
            if(pDst[j]!=pSrc[i+j])  
            break;  
        }  
        if(pDst[j]=='\0')  
            return i;  
    }  
    return -1;  
}  
