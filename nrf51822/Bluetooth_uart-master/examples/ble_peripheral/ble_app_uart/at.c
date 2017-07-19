//AT 命令处理
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_uart.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_gpiote.h"
#include "nrf_ppi.h"
#include "string.h"
#include "at.h"
#include <stdarg.h>
#include "Ble_FirmwareInfo.h"


#define 	ATCMD_PINNO                (28)
#define 	BLE_CONNECTSTATUS_PINNO    (29u)



#define     ATCMD_QUERY_BLEMAC_HEADER	    "AT+LEMAC?\r\n"     
#define     ATCMD_QUERY_SOFTVER_HEADER		"AT+VER?\r\n"
#define     ATCMD_QUERY_BLESSID_HEADER		"AT+LENAME?\r\n"
#define     ATCMD_SET_BLESSID_HEADER        "AT+LENAME="
#define     ATCMD_SET_URATE_HEADER          "AT+URATE="
#define     ATMCD_QUERY_URATE_HEADER        "AT+URATE?\r\n"



extern BLE_CFG_FIRMWAREINFO          ble_cfg_firmwareinfo;



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

void ATcmd_Pin_Configuration(void)
{
        do 
        {                                                  
            nrf_gpio_cfg_input(ATCMD_PINNO,GPIO_PIN_CNF_PULL_Pullup);
        }while (0);
        
}

uint8_t  Check_ATcmd_Pin_Level  (void)
{
        return  nrf_gpio_pin_read(ATCMD_PINNO);
}

void    Ble_ConnectStatusPin_Configuration(void)
{
        nrf_gpio_cfg_output(BLE_CONNECTSTATUS_PINNO);
}

void    Ble_ConnectStatus_SetHigh(void)
{
        nrf_gpio_pin_set(BLE_CONNECTSTATUS_PINNO);
}

void    Ble_ConnectStatus_SetLow(void)
{
        nrf_gpio_pin_clear(BLE_CONNECTSTATUS_PINNO);
}

void    Test_Pin_Invert(void)
{
        // nrf_gpio_pin_toggle(BLE_CONNECTSTATUS_PINNO);
}


void    Ble_ATcmd_Parser(uint8_t *pbuf,uint16_t len)
{
        if(StringFind((char*)pbuf,ATCMD_QUERY_BLEMAC_HEADER)==0)
        {
            Ble_ATcmd_LEMAC_Handler(pbuf,  len);
        }  

        if(StringFind((char*)pbuf,ATCMD_QUERY_SOFTVER_HEADER)==0)
        {
            Ble_ATcmd_QuerySoftwareVerReply_Handler( pbuf,len);
        }

        if(StringFind((char*)pbuf,ATCMD_QUERY_BLESSID_HEADER)==0)
        {
             Ble_ATcmd_LENAMEReply_Handler(pbuf,len);
        }

        if(StringFind((char*)pbuf,ATCMD_SET_BLESSID_HEADER)==0)
        {
            Ble_ATcmd_LENAME_Handler(&pbuf[sizeof(ATCMD_SET_BLESSID_HEADER)],len-sizeof(ATCMD_SET_BLESSID_HEADER));
        }

        if(StringFind((char*)pbuf,ATCMD_SET_URATE_HEADER)==0)
        {
            
        }          
}

void	Ble_ATcmd_LENAME_Handler(uint8_t* pbuf, uint8_t len)
{
        uint8_t temp_len=0,temp[32]={0},i=0;
         
        if(len>12)
        {
         len=12;   
        }
        memcpy(ble_cfg_firmwareinfo.lename_ssid,pbuf,len);
        FirmwareInfo_upgrade(&ble_cfg_firmwareinfo);
        temp_len=sprintf((char*)temp,"OK\r\n");
        for(i=0;i<temp_len;i++)
        {
          while(app_uart_put(temp[i]) != NRF_SUCCESS); 
        }
}

void    Ble_ATcmd_LENAMEReply_Handler(uint8_t* pbuf, uint8_t len)
{
        char temp[64]={0},ssid_len=0,temp_len=0;
        uint8_t i=0;
        
        FirmwareInfo_Read(&ble_cfg_firmwareinfo);
        temp_len=sprintf((char*)temp,"+LENAME:");
        memcpy(&temp[temp_len],ble_cfg_firmwareinfo.lename_ssid,12);
        ssid_len=temp_len+12+sprintf((char*)&temp[temp_len+12],"\r\n");
        for(i=0;i<ssid_len;i++)
        {
           app_uart_put(temp[i]);
        }
}

void 	Ble_ATcmd_LEMAC_Handler(uint8_t* pbuf, uint8_t len)
{
        uint32_t           i=0;
        ble_gap_addr_t     p_addr;
        uint8_t            mac_addr[31]={0},mac_addr_len=0;
    
        sd_ble_gap_address_get(&p_addr);
        mac_addr_len=sprintf((char*)mac_addr,"+LEMAC:%x:%x:%x:%x:%x:%x\r\n",p_addr.addr[0],p_addr.addr[1],p_addr.addr[2],p_addr.addr[3],p_addr.addr[4],p_addr.addr[5]);
        for(i=0;i<mac_addr_len;i++)
        {
            app_uart_put(mac_addr[i]);
        }
}

void	Ble_ATcmd_QuerySoftwareVerReply_Handler(uint8_t* pbuf, uint8_t len)
{
        char temp[64]={0};
        uint8_t softver_len=0,i=0,temp_len=0;
        
        softver_len=sprintf(temp,"VER:");
        memcpy(&temp[softver_len],FIRMWARE_VERSION,sizeof(FIRMWARE_VERSION));
        temp_len=sprintf(&temp[softver_len+sizeof(FIRMWARE_VERSION)],"\r\n");
        softver_len=softver_len+sizeof(FIRMWARE_VERSION)+temp_len;
        for(i=0;i<softver_len;i++)
        {
           app_uart_put(temp[i]);
        }
}

void    Ble_ATcmd_SetURATE_Handler(uint8_t* pbuf, uint8_t len)
{
        char temp[64]={0};
        uint8_t return_len=0,i=0;
        
        return_len=sprintf(temp,"ATCMD_SET_URATE\r\n");
        for(i=0;i<return_len;i++)
        {
           app_uart_put(temp[i]);
        }
}

void    Ble_ATcmd_QueyURATE_Handler(uint8_t* pbuf, uint8_t len)
{



}

void uart_printf(char* fmt,...)
{
    uint8_t i=0;

    va_list ap;
    char string[128];
    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    while(string[i])
    {
        while(app_uart_put(string[i]) != NRF_SUCCESS); 
        i++;
    }
    va_end(ap);
}
