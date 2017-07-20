

#ifndef APPTASK_BROADCOM_BLE_H

#define APPTASK_BROADCOM_BLE_H


#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "Task.h"


typedef __packed struct 
{
 uint8_t opcode;
 uint8_t ble_mac[12];
}BLE_PAIR;

typedef __packed struct 
{
 uint8_t   current_mode;	
 uint8_t   module_startup_flag;	
 uint8_t   pair_status;	
 uint8_t   pair_request;
 BLE_PAIR  pair_data;
	
}BLE_PAIR_HANDLE;


         void AppTask_Ble_Pair_Creat(void);
 static  void ble_pair_task(void *pvParameters);

        void Ble_Pair_Broadcast_Cmd_Send(void);
        void BleRecived_Slave_Broadcast_cmd_Ack(uint8_t* src);
        void BleRecieved_Mater_Broadcast_cmd(uint8_t* src);
 static void BleSlave_Send_Broadcast_cmd_Ack(void);
        void BleCmd_Request_Notifiacation(void);

#endif
