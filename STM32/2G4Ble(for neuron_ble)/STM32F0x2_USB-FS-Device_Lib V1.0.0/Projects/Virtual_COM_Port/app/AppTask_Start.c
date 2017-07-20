


#include "AppTask_Start.h"

#include "Ring_Buf.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "stm32f0xx.h"
#include "Board.h"
#include "AppTask_Uart1.h"
#include "serialManager.h"
#include "AppTask_Uart2.h"
#include "AppTask_Uart3.h"
#include "Broadcom_Ble.h"
#include "AppTask_BroadcomBle.h"
#include "hw_config.h"
#include "Read_ModuleID.h"

extern BLE_CFGINFO          Ble_Cfg_Mac;

PASSTHROUGH_HANDLE          Passthrough_Handle;

 void   AppTask_Start_Creat(void)
 {

    xTaskCreate(Mcu_startup_task,"mcu sartup cfg",128,NULL,0,NULL);	
 }

void Mcu_startup_task(void *pvParameters)
{   
	Passthrough_Handle.uart_online_starus.uart1_online_singal=0;
	Passthrough_Handle.uart_online_starus.uart2_online_singal=0;
	Passthrough_Handle.uart_online_starus.uart3_online_singal=0; // defulat status
    Passthrough_Handle.uart_online_starus.usb_online_singal=0;
    
    Borad_Start_Init();
    vTaskDelay( 10/portTICK_RATE_MS );
    USB_CDC_Init();
  
    //ModuleID_ADPin_Init();
    vTaskDelay( 100/portTICK_RATE_MS );
    AppTask_Uart1_Create();
   	AppTask_Uart2_Create();
    AppTask_Uart3_Create();
    AppTask_Ble_Pair_Creat();
    Ble_Info_Init();
	while(1)
	{
          vTaskDelay( 500/portTICK_RATE_MS );
		 // Ble_Pair_Broadcast_Cmd_Send();
		  Module_Mode_Status_Display();
		 // Send_Test_Cmd();   
         // Ble_ATCmd_Tick();
          USB_Printf("usb com is running\r\n");
	}
}

void  Send_Test_Cmd(void)
{
    uint8_t test_cmd_buf[10]={0xf0,0xff,0xff,0x01,0x02,0x03,0x04,0x05,0x0d,0xF7};     
    if(Passthrough_Handle.uart_online_starus.uart1_online_timerout_cnt==0)
    {
       Uart_Send(USART1,test_cmd_buf,10);	
    }	

    if(Passthrough_Handle.uart_online_starus.uart2_online_timerout_cnt==0)
    {
        Uart_Send(USART2,test_cmd_buf,10);
    }	 
}

void   Check_Connect_Mode  (PASSTHROUGH_HANDLE* passthrough_handle)
{
// if(passthrough_handle->passthrough_dir!=STOP_UART3_PASSTHROUGH)	
//  {
//    if(passthrough_handle->uart_online_starus.usb_online_singal)  
//        {
//             passthrough_handle->passthrough_dir=UART3_TO_USB;
//        }
//	else if ((passthrough_handle->uart_online_starus.uart1_online_singal)&&(passthrough_handle->uart_online_starus.uart3_online_singal))
//		{
//			passthrough_handle->passthrough_dir=UART1_TO_UART3;
//		}
//	else if((passthrough_handle->uart_online_starus.uart2_online_singal)&&(passthrough_handle->uart_online_starus.uart3_online_singal))
//		{
//			passthrough_handle->passthrough_dir=UART2_TO_UART3;
//		}
//	else if (passthrough_handle->uart_online_starus.uart1_online_singal)
//		{
//		    passthrough_handle->passthrough_dir=UART1_TO_UART3; 
//		}
//	else if (passthrough_handle->uart_online_starus.uart2_online_singal)
//		{
//			passthrough_handle->passthrough_dir=UART2_TO_UART3;
//		}
//	else if (passthrough_handle->uart_online_starus.uart3_online_singal)
//        {
//            passthrough_handle->passthrough_dir=UART3_TO_UART1andUart2;
//        }		
//
    if(passthrough_handle->uart_online_starus.usb_online_singal)    // this code is just for bluetooth
    {
        passthrough_handle->passthrough_dir=UART2_TO_USB;
    }
    else if(passthrough_handle->uart_online_starus.uart3_online_singal) 
    {
        passthrough_handle->passthrough_dir=UART3_TO_UART1andUart2;
    }
    else 
    {
        passthrough_handle->passthrough_dir=UART2_TO_USB;
    }
}

void   Module_Mode_Status_Display(void)
{
 if(Ble_Cfg_Mac.ble_mode==1)  // master(Ap mode )
 { 
	 if(Passthrough_Handle.uart_online_starus.uart3_online_singal)
	 {
         Board_Led_on(0);
	 }
	 else
	 {
		 Board_Led_Toggle(0);  // 0: blue 1:green 2: red
	 }
     
     Board_Led_off(1);
     Board_Led_off(2); 
     
 }
 else if(Ble_Cfg_Mac.ble_mode==0)
 {
	 if(Passthrough_Handle.uart_online_starus.uart3_online_singal)
	 {
        Board_Led_on(1);
	 }
	 else
	 {
        Board_Led_Toggle(1);  // station
	 }
     
     Board_Led_off(0);
     Board_Led_off(2); 
 }
 else
 {
    Board_Led_Toggle(2);  // red flash
    Board_Led_off(0);
    Board_Led_off(1);  
 }
}

void Com_Connect_Timeout_Isr(PASSTHROUGH_HANDLE* passthrough_handle)
{
    if(passthrough_handle->uart_online_starus.uart1_online_timerout_cnt>0)
            {
                passthrough_handle->uart_online_starus.uart1_online_timerout_cnt--;
            }
    else
            {
                passthrough_handle->uart_online_starus.uart1_online_singal=0;
            }

    if(passthrough_handle->uart_online_starus.uart2_online_timerout_cnt>0)
           {
                passthrough_handle->uart_online_starus.uart2_online_timerout_cnt--;
           }
    else
           {
                passthrough_handle->uart_online_starus.uart2_online_singal=0;
           }
         
    if(passthrough_handle->uart_online_starus.uart3_online_timerout_cnt>0)
           {
                passthrough_handle->uart_online_starus.uart3_online_timerout_cnt--;
           }
    else
           {
                passthrough_handle->uart_online_starus.uart3_online_singal=0;
           }
}

void vApplicationTickHook (void)
{
     Com_Connect_Timeout_Isr(&Passthrough_Handle);
     Ble_ConnectStatus_Monitor();
}
