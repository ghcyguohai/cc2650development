
#ifndef AppTask_Start_H
#define AppTask_Start_H

#include "stdint.h"
#include "stdio.h"

typedef enum
{
 UART1_TO_UART2=0,
 UART1_TO_UART3,
 UART2_TO_UART3,
 UART3_TO_USB,
 UART2_TO_USB,
 UART1_TO_UART2andUART3,
 UART2_TO_UART3andUART1,
 UART3_TO_UART1andUart2,	
 UART1_TO_UART2_STOP,  // stop the passthrough btween uart 1 and uart3
 UART1_TO_UART3_STOP,
 UART2_TO_UART3_STOP,
 STOP_UART3_PASSTHROUGH,
 LOOP_BACK,

}TRANS_DIR;

typedef struct 
{
 uint16_t uart1_online_timerout_cnt;
 uint16_t uart2_online_timerout_cnt;	
 uint16_t uart3_online_timerout_cnt;
 uint8_t  uart1_online_singal;
 uint8_t  uart2_online_singal; 	
 uint8_t  uart3_online_singal;
 uint8_t  usb_online_singal;
}CON_ONLINE_STATUS;

typedef struct 
{
 CON_ONLINE_STATUS uart_online_starus;
 TRANS_DIR         passthrough_dir;
}PASSTHROUGH_HANDLE;


       void   AppTask_Start_Creat(void);
       void   Borad_Start_Init   (void);
       void   Mcu_startup_task   (void *pvParameters);
       void   Send_Test_Cmd(void);
       void   Check_Connect_Mode  (PASSTHROUGH_HANDLE* passthrough_handle);
       void   Module_Mode_Status_Display(void);
       void   Com_Connect_Timeout_Isr(PASSTHROUGH_HANDLE* passthrough_handle);
#endif
