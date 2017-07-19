

#ifndef BLE_CONFIGSERVICE_H
#define BLE_CONFIGSERVICE_H

#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "ble.h"



typedef void (*CUSTOME_SERVICE_HANBLER_HANDLER_PRT) (uint8_t data);


typedef struct
{
    uint8_t                                             uuid_type;           /**< UUID type for the LED Button Service. */ 
    uint16_t                                            conn_handle;              
    uint16_t                                            service_handle;           
    ble_gatts_char_handles_t                            setgpio_level_char1_handlers;   
    ble_gatts_char_handles_t                            setgpio_level_char2_handlers;  
    ble_gatts_char_handles_t                            setgpio_level_char3_handlers;
    ble_gatts_char_handles_t                            setgpio_level_char4_handlers;    
    ble_gatts_char_handles_t                            setuart_parity_char5_handlers;   
    ble_gatts_char_handles_t                            setgpio_setadvinterval_char6_handlers;  
    ble_gatts_char_handles_t                            setgpio_setlowpowermode_char7_handlers;  
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setgpio_level_char1_fun;
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setgpio_level_char2_fun;
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setgpio_level_char3_fun;
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setgpio_level_char4_fun;   
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setuart_parity_char5_fun;  
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setadv_interval_char6_fun;
    CUSTOME_SERVICE_HANBLER_HANDLER_PRT                 setble_lowpowermode_char7_fun;
}BLE_CFG_SERVICE;  


           void  CustomSevice_init(void);
static uint32_t  CustomeService_SetGPIO_char1_add(void);
static uint32_t  CustomeService_SetGPIO_char2_add(void);
static uint32_t  CustomeService_SetGPIO_char3_add(void);
static uint32_t  CustomeService_SetGPIO_char4_add(void);
static uint32_t  CustomeService_SetUARTPARITY_char5_add(void);
static uint32_t  CustomeService_SET_ADVINTERVAL_char6_add(void);
static uint32_t  CustomeService_SET_LOWPOWERMODE_char7_add(void);



static     void  SetGPIO_Char1_Function(uint8_t data);
static     void  SetGPIO_Char2_Function(uint8_t data);
static     void  SetGPIO_Char3_Function(uint8_t data);
static     void  SetGPIO_Char4_Function(uint8_t data);
static     void  SetUAUT_Parity_Char5_Function(uint8_t data);
static     void  SetADV_Interval_Char6_Function(uint8_t data);
static     void  SetBLE_Lowpowermode_Char7_Function(uint8_t data);



void   CustomService_ble_evt_handle(ble_evt_t * p_ble_evt);

#endif
