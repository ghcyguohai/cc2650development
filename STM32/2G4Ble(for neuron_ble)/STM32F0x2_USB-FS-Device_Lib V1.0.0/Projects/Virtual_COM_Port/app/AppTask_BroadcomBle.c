

#include  "Broadcom_Ble.h"

#include "Broadcom_Ble.h"
#include "AppTask_BroadcomBle.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "Old_Protocol.h"
#include "event_groups.h"
#include "serialManager.h"

#define OPCODE_PAIR       (0x01u)
#define OPCODE_PAIR_ACK   (0x02u)



#define  BIT(x)         (1u<<x)
#define  COM_CMD_FLAG    BIT(0)


static EventGroupHandle_t   BLE_Pair_EventGroup_Handle = NULL;
       BLE_PAIR_HANDLE      Ble_Pair_Handle;

extern PARSER_HANDLE   Uart1_Protocol_Unpack_Handle;
extern PARSER_HANDLE   Uart2_Protocol_Unpack_Handle;
extern PARSER_HANDLE   Uart3_Protocol_Unpack_Handle;
extern BLE_CFGINFO     Ble_Cfg_Mac;

 void   AppTask_Ble_Pair_Creat(void)
 {
	 Ble_Pair_Handle.module_startup_flag=1; // this flag, indicate: the module has startup and the module has entered into passthrough mode,
	 BLE_Pair_EventGroup_Handle=xEventGroupCreate();
	 if(BLE_Pair_EventGroup_Handle!=NULL)
	 {
        xTaskCreate(ble_pair_task,"ble pair",128,NULL,5,NULL);	
	 }
 }
 
 static  void ble_pair_task(void *pvParameters)
 {
   EventBits_t  Com_Cmd_Flag;
	 
	while(1)
	 { 
	  Com_Cmd_Flag =xEventGroupWaitBits(
                                          BLE_Pair_EventGroup_Handle,
                                          COM_CMD_FLAG,
                                          pdTRUE, 
                                          pdFALSE,
                                          portMAX_DELAY
                                        );  // portMAX_DELAY: indicate: the task will wait forever  

		 if(Com_Cmd_Flag&COM_CMD_FLAG)   
		 {
			// if(WIFI_Pair_Handle.pair_status==0) // 0; pair do not finish. 1: pair finish
           {				
				if(Ble_Pair_Handle.pair_request==1)
				{	
                        USB_Printf("recieved handshake events \r\n");
						if(Ble_Pair_Handle.pair_data.opcode==OPCODE_PAIR_ACK)  // recieved  slave ack. from uart1
    						{
    								Ble_Pair_Handle.pair_status=1; // pair nishan
    								Ble_Pair_Handle.pair_request=0;// clear the pair request
    								USB_Printf("recieved slave pair ack \r\n");
                                    USB_Printf("recieved slave mac is %d%c%c%c%c%c%c%c%c%c%c%c%c\r\n",
                                               Ble_Pair_Handle.pair_data.ble_mac[0], Ble_Pair_Handle.pair_data.ble_mac[1],
                                               Ble_Pair_Handle.pair_data.ble_mac[2], Ble_Pair_Handle.pair_data.ble_mac[3],
                                               Ble_Pair_Handle.pair_data.ble_mac[4], Ble_Pair_Handle.pair_data.ble_mac[5],
                                               Ble_Pair_Handle.pair_data.ble_mac[6], Ble_Pair_Handle.pair_data.ble_mac[7],
                                               Ble_Pair_Handle.pair_data.ble_mac[8], Ble_Pair_Handle.pair_data.ble_mac[9],
                                               Ble_Pair_Handle.pair_data.ble_mac[10], Ble_Pair_Handle.pair_data.ble_mac[11]
                                ); 
     
    								if(Ble_Cfg_Mac.ble_mode==0)  // if the crrent mode is slave,change self mode to master mode and save ble info
        							{
        									Ble_Cfg_Mac.ble_mode= 1;   // 1: master mode 
                                            memcpy(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12);  
                                            if(Ble_ATCmdPassthrough_Send()==0) // if set ok
        									 {
                                                 USB_Printf("reconfig ble mac   successfully \r\n");
                                                 Ble_Cfg_Mac.pair_flag=1;
                                                 Ble_Info_Write(&Ble_Cfg_Mac);	
        										 Ble_Pair_Handle.pair_status=1;      //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
        										 Ble_Pair_Handle.pair_request=0;     // clear pair request flag		  
                                             }
                                             else
                                             {
                                                 USB_Printf("reconfig ble mac failed \r\n");
                                                 Ble_Cfg_Mac.pair_flag=0; 
                                             }
                                     }
                                    else if (Ble_Cfg_Mac.ble_mode==1)  // current mode is master, save and set slave info
                                    {
                                            if(memcmp(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12)!=0)	 // if the mac is different		
                                            {
                                              USB_Printf("get new mac\r\n");  
                                              memcpy(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12);  
                                              if(Ble_ATCmdPassthrough_Send()==0)
                                              {
                                                 Ble_Cfg_Mac.pair_flag=1;
                                                 Ble_Info_Write(&Ble_Cfg_Mac);	
                                                 Ble_Pair_Handle.pair_status=1;      //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
                                                 Ble_Pair_Handle.pair_request=0;     // clear pair request flag		  
                                                 USB_Printf("reconfig target mac successfully  \r\n");
                                              }
                                              else   
                                              {
                                                 USB_Printf("reconfig target mac failed\r\n"); 
                                                 Ble_Cfg_Mac.pair_flag=0;
                                              } 
                                            
                                            }
                                            else
                                            {
                                                USB_Printf("the ble mac is same,do not anythin\r\n"); 
                                                Ble_Pair_Handle.pair_status=1;      //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
                                                Ble_Pair_Handle.pair_request=0;     // clear pair request flag		  
                                            }
                                    }
    							 USART_Cmd(USART2,ENABLE);            //  enable uart2	: retore the trans of uart2							
    				        }
					    else    // enable uart2 drectly
    						{
    							USART_Cmd(USART2,ENABLE);            //  enable uart2	: retore the trans of uart2
    						}						
					
        				if(Ble_Pair_Handle.pair_data.opcode==OPCODE_PAIR)  // recieved master pair cmd, from uart2
            				 {	
                                 USB_Printf("recieved master pair command  \r\n"); 
                                 USB_Printf("recieved master mac is=%c%c%c%c%c%c%c%c%c%c%c%c\r\n",
                                               Ble_Pair_Handle.pair_data.ble_mac[0], Ble_Pair_Handle.pair_data.ble_mac[1],
                                               Ble_Pair_Handle.pair_data.ble_mac[2], Ble_Pair_Handle.pair_data.ble_mac[3],
                                               Ble_Pair_Handle.pair_data.ble_mac[4], Ble_Pair_Handle.pair_data.ble_mac[5],
                                               Ble_Pair_Handle.pair_data.ble_mac[6], Ble_Pair_Handle.pair_data.ble_mac[7],
                                               Ble_Pair_Handle.pair_data.ble_mac[8], Ble_Pair_Handle.pair_data.ble_mac[9],
                                               Ble_Pair_Handle.pair_data.ble_mac[10], Ble_Pair_Handle.pair_data.ble_mac[11]
                                ); 
                                    if(Ble_Cfg_Mac.ble_mode==1)              // if cureent mode is master,change self mode to slave mode,and we should save the pair info (such as ssid\password and port)
                                    {
                                        Ble_Cfg_Mac.ble_mode=0; // change selt to salve mode 
                                        memcpy(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12);
                                         USB_Printf("switch to slave mode  \r\n"); 
                                        if(Ble_ATCmdPassthrough_Send()==0)
                                        {
                                             USB_Printf("slave mode config successfully  \r\n"); 
                                            Ble_Cfg_Mac.pair_flag=1;
                                            Ble_Info_Write(&Ble_Cfg_Mac);
                                            Ble_Pair_Handle.pair_status=1; //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
                                            Ble_Pair_Handle.pair_request=0; // clear pair request flag							
                                            BleSlave_Send_Broadcast_cmd_Ack(); // tell the master, pair finish                                            
                                        }
                                        else
                                        { 
                                            USB_Printf("slave mode config failed  \r\n"); 
                                            Ble_Cfg_Mac.pair_flag=0;   // pair faile
                                        }
                                    }
                                    else if(Ble_Cfg_Mac.ble_mode==0)    // if current mode is slave ,chen check mac password port 
                                    {
                                         USB_Printf("current mode is slave  \r\n"); 
                                        if(memcmp(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12)!=0)	 // if the mac is different				 					
                                        {
                                           USB_Printf("get new mac  \r\n"); 
                                           memcpy(Ble_Cfg_Mac.ble_targetmac,Ble_Pair_Handle.pair_data.ble_mac,12);
                                           if(Ble_ATCmdPassthrough_Send()==0)
                                           {
                                                USB_Printf("slave config sucessfully  \r\n"); 
                                                 Ble_Cfg_Mac.pair_flag=1; 
                                                 Ble_Pair_Handle.pair_status=1; //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
                                                 Ble_Pair_Handle.pair_request=0; // clear pair request flag	
                                                 Ble_Info_Write(&Ble_Cfg_Mac);  // save ble info        
                                               
                                                 memcpy(Ble_Pair_Handle.pair_data.ble_mac,Ble_Cfg_Mac.ble_selfmac,12);   // upload self mac value
                                                 BleSlave_Send_Broadcast_cmd_Ack(); // tell the master, pair finish
                                           }
                                           else
                                           {
                                                USB_Printf("slave config failed  \r\n"); 
                                                Ble_Cfg_Mac.pair_flag=0;
                                           }
                                        }
                                        else
                                        {
                                           USB_Printf(" the mac address is same  \r\n");
                                           Ble_Pair_Handle.pair_status=1; //pair finish, flag it to avoid the enter into Set_Esp8266_State() again.
                                           Ble_Pair_Handle.pair_request=0; // clear pair request flag
                                           memcpy(Ble_Pair_Handle.pair_data.ble_mac,Ble_Cfg_Mac.ble_selfmac,12);   // up self mac value                                           
                                           BleSlave_Send_Broadcast_cmd_Ack(); // tell the master, pair finish
                                        } 
                                    }			 
    
                                USART_Cmd(USART1,ENABLE); //  enable uart2	: retore the trans of uart2					
                            }
                        else
                            {
                                USART_Cmd(USART1,ENABLE); //  enable uart1 directly
                            }
		          }
		  }
	   }			
	}
 }
 
 extern PASSTHROUGH_HANDLE   Passthrough_Handle;
 void Ble_Pair_Broadcast_Cmd_Send(void)
 {
     BLE_PAIR broad_data;
//	 if(WIFI_Pair_Handle.current_mode==1)  //  whether the curent mode is master or slave.the module should send broadcast cmd to uart1;
		 {	 
			if((Ble_Pair_Handle.pair_status!=1)&&(Passthrough_Handle.uart_online_starus.uart3_online_singal!=1))  // if pair finish  . the module will stop send broadcast cmd  
			{
                USB_Printf("send pair command \r\n");
				broad_data.opcode=OPCODE_PAIR;
                memcpy(broad_data.ble_mac,Ble_Cfg_Mac.ble_selfmac,12);
				Cmd_Pack_Send(Uart1_Protocol_Unpack_Handle.ring_buf->uart,(uint8_t*)&broad_data,sizeof(BLE_PAIR));
			}
	   }
}
 
 void BleRecived_Slave_Broadcast_cmd_Ack(uint8_t* src)  
 {
  BLE_PAIR* temp=(BLE_PAIR*)src;
	 
	if( temp->opcode==OPCODE_PAIR_ACK)
	{
         Ble_Pair_Handle.current_mode=1;   // if recieved slave ack ,chang mode to master mode 
         Ble_Pair_Handle.pair_request=1;	
         Ble_Pair_Handle.pair_data.opcode=temp->opcode;  // just check the ack.
         memcpy(Ble_Pair_Handle.pair_data.ble_mac,temp->ble_mac,12);
	} 
 }

 void BleRecieved_Mater_Broadcast_cmd(uint8_t* src)
 {
   BLE_PAIR* temp=(BLE_PAIR*)&src[0];

	 if( temp->opcode==OPCODE_PAIR)  
	{
         Ble_Pair_Handle.current_mode=0;  // if recieved master broadcast cmd , change self mode from master mode to slave mode 
         Ble_Pair_Handle.pair_request=1;	
         Ble_Pair_Handle.pair_data.opcode=temp->opcode;	
         memcpy(Ble_Pair_Handle.pair_data.ble_mac,temp->ble_mac,12);
	}
 }
 
static void BleSlave_Send_Broadcast_cmd_Ack(void)  
 {
	Ble_Pair_Handle.pair_data.opcode=OPCODE_PAIR_ACK;
    Cmd_Pack_Send(Uart2_Protocol_Unpack_Handle.ring_buf->uart,(uint8_t*)&Ble_Pair_Handle.pair_data,sizeof(BLE_PAIR)); 
 }
 
void  BleCmd_Request_Notifiacation(void)
{
    xEventGroupSetBits(
                            BLE_Pair_EventGroup_Handle,
                            COM_CMD_FLAG
                       ); 
}
