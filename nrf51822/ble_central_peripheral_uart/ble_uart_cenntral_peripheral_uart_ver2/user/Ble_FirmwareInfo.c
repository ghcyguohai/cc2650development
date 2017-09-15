


#include "Ble_FirmwareInfo.h"
#include "pstorage.h"
#include "string.h"


#define   MCU_FLASH_PAGE_SIZE       NRF_FICR->CODEPAGESIZE          //1024
#define   MCU_FLASH_PAGENUM         (NRF_FICR->CODESIZE - 1)        //256-1=255 Use last page in flash




      BLE_CFG_FIRMWAREINFO          ble_cfg_firmwareinfo;
const BLE_CFG_FIRMWAREINFO          defualt_firmwareinfo=
                                                         {
                                                            {'M','a','k','e','b','l','o','c','k','_','L','E'
                                                            },
                                                            UART_BRUADRATE_115200
                                                         };  

                                                         
static       pstorage_handle_t          firmwareinfo_handle;
static       pstorage_module_param_t    param;

uint32_t  Get_Flash_PageSize(void)
{
    return MCU_FLASH_PAGE_SIZE;
}

uint32_t  Get_Flash_PageNums(void)
{
    return MCU_FLASH_PAGENUM;
}

uint32_t FirmwareInfo_Addr(void)
{
    return (0x18000+MCU_FLASH_PAGE_SIZE*64);
}

void FirmwareInfo_Default(void)
{
    uint32_t result=0;
    
    param.block_size  = sizeof(BLE_CFG_FIRMWAREINFO);                  
    param.block_count = 1;                   //Select 1 blocks, total of sizeof(BLE_CFG_FIRMWAREINFO) 
    param.cb          = example_cb_handler;   //Set the pstorage callback handler
    result = pstorage_init();
    
    result = pstorage_register(&param, &firmwareinfo_handle);
    if(result!=NRF_SUCCESS)
    {
        
    }
    FirmwareInfo_Read(&ble_cfg_firmwareinfo);
}

uint8_t FirmwareInfo_upgrade(BLE_CFG_FIRMWAREINFO* src_ble_cfg_firmwareinfo)
{
    uint32_t     result=0;
    
    result=pstorage_block_identifier_get(&firmwareinfo_handle,0, &firmwareinfo_handle);
    result=pstorage_clear(&firmwareinfo_handle, sizeof(BLE_CFG_FIRMWAREINFO));
    result=pstorage_store(&firmwareinfo_handle, (uint8_t*)src_ble_cfg_firmwareinfo, sizeof(BLE_CFG_FIRMWAREINFO), 0);     //Write to flash

    return  result;
}

static void example_cb_handler(pstorage_handle_t  * handle, uint8_t op_code,uint32_t  result, uint8_t * p_data,uint32_t data_len)
{	
		switch(op_code)
		{
			case PSTORAGE_LOAD_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
								 
				 }
				 else
				 {
					
				 }
				 break;
			case PSTORAGE_STORE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
			
				 }
				 else
				 {
				
				 }
				 break;				 
			case PSTORAGE_UPDATE_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
			
				 }
				 else
				 {
			
				 }
				 break;
			case PSTORAGE_CLEAR_OP_CODE:
				 if (result == NRF_SUCCESS)
				 {
					
				 }
				 else
				 {
				
				 }
				 break;			 
		}			
}

void     FirmwareInfo_Read(BLE_CFG_FIRMWAREINFO* dst_ble_cfg_firmwareinfo)
{
        pstorage_block_identifier_get(&firmwareinfo_handle,0, &firmwareinfo_handle);
    	pstorage_load((uint8_t *)dst_ble_cfg_firmwareinfo,&firmwareinfo_handle ,sizeof(BLE_CFG_FIRMWAREINFO), 0);				 //Read from flash, only one block is allowed for each pstorage_load command
}

