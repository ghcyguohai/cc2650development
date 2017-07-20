


#include "usb_core.h"
#include "usbd_conf.h"
#include "usbd_core.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "hw_config.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"


USB_CORE_HANDLE  USB_Device_dev ;

uint8_t usb_send_finish_flag=0;

void USB_CDC_Init(void)
{
  USB_pullup_High();
  USBD_Init(&USB_Device_dev,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
}

void USB_CDC_Send(uint8_t* src,uint16_t len)
{
  uint16_t send_64bytes_cnt=0,send_bytes_cnt=0; 
  uint16_t send_index=0;
  uint8_t  send_timeout_cnt=0;    
    
  send_64bytes_cnt=len/CDC_DATA_MAX_PACKET_SIZE;
  send_bytes_cnt=len%CDC_DATA_MAX_PACKET_SIZE;
 
 
      while(send_64bytes_cnt)
      {
        usb_send_finish_flag=0;
        /* send  packet to PMA*/
        UserToPMABufferCopy(src+send_index, BULK_IN_TX_ADDRESS, CDC_DATA_MAX_PACKET_SIZE);
        SetEPTxCount(ENDP1, CDC_DATA_MAX_PACKET_SIZE);
        SetEPTxValid(ENDP1);
        send_index+=CDC_DATA_MAX_PACKET_SIZE;
        send_64bytes_cnt--;
          
        do
        {
         usb_send_finish_flag++;
         vTaskDelay( 1/portTICK_RATE_MS );
        }while((usb_send_finish_flag==0)&&(send_timeout_cnt<100));     
      }

      if(send_bytes_cnt)
      {
        usb_send_finish_flag=0;
        UserToPMABufferCopy(src+send_index, BULK_IN_TX_ADDRESS, send_bytes_cnt);
        SetEPTxCount(ENDP1, send_bytes_cnt);
        SetEPTxValid(ENDP1);
        do
        {
         usb_send_finish_flag++;
         vTaskDelay( 1/portTICK_RATE_MS );
        }while((usb_send_finish_flag==0)&&(send_timeout_cnt<100));        
      }
  
}

void USB_CDC_int_tx_isr(void)
{



}

void USB_CDC_Recieve(uint8_t* src,uint16_t len)
{
    
}

