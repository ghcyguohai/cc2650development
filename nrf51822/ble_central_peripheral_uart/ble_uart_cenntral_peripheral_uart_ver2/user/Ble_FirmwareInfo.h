

#ifndef BLE_FIRMWAREINFO_H

#define BLE_FIRMWAREINFO_H

#include "stdint.h"
#include "nrf_error.h"
#include "pstorage_platform.h"
#include "pstorage.h"

#define FIRMWARE_VERSION   "VER_00_00_01"




typedef enum 
{

    UART_BRUADRATE_9600=(uint32_t)0,
    UART_BRUADRATE_19200,
    UART_BRUADRATE_38400,
    UART_BRUADRATE_57600,
    UART_BRUADRATE_115200
}UART_BRUADRATE_CFG;

typedef struct _packed 
{
    uint8_t            lename_ssid[12];
    uint32_t           uart_bruadte_cfg_value;
}BLE_CFG_FIRMWAREINFO;


uint32_t  Get_Flash_PageSize(void);
uint32_t  Get_Flash_PageNums(void);
uint32_t  FirmwareInfo_Addr(void); 

void        FirmwareInfo_Default(void);
uint8_t     FirmwareInfo_upgrade(BLE_CFG_FIRMWAREINFO* src_ble_cfg_firmwareinfo);
static void example_cb_handler(pstorage_handle_t  * handle,uint8_t op_code,uint32_t  result,uint8_t * p_data,uint32_t  data_len);
void        FirmwareInfo_Read(BLE_CFG_FIRMWAREINFO* dst_ble_cfg_firmwareinfo);

#endif 
