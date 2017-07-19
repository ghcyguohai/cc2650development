

#include "ble_configservice.h"
#include "at.h"




#define  BLE_SET_SERVICE_UUID                           0xFFE4

#define  BLE_UUID_CHARACTERISTIC1_SETGPIO_LEVEL         0xFFE5
#define  BLE_UUID_CHARACTERISTIC2_SETGPIO_LEVEL         0xFFE6  
#define  BLE_UUID_CHARACTERISTIC3_SETGPIO_LEVEL         0xFFE7
#define  BLE_UUID_CHARACTERISTIC4_SETGPIO_LEVEL         0xFFE8
#define  BLE_UUID_CHARACTERISTIC5_SETUART_PARITY        0xFFE9
#define  BLE_UUID_CHARACTERISTIC6_SET_ADVINTERVAL       0xFFEA
#define  BLE_UUID_CHARACTERISTIC7_SET_LOWPOWERMODE      0xFFEB


BLE_CFG_SERVICE  CustomService;


//typedef struct
//{
//  ble_gatt_char_props_t       char_props;               /**< Characteristic Properties. */
//  ble_gatt_char_ext_props_t   char_ext_props;           /**< Characteristic Extended Properties. */
//  uint8_t                    *p_char_user_desc;         /**< Pointer to a UTF-8 encoded string (non-NULL terminated), NULL if the descriptor is not required. */
//  uint16_t                    char_user_desc_max_size;  /**< The maximum size in bytes of the user description descriptor. */
//  uint16_t                    char_user_desc_size;      /**< The size of the user description, must be smaller or equal to char_user_desc_max_size. */ 
//  ble_gatts_char_pf_t*        p_char_pf;                /**< Pointer to a presentation format structure or NULL if the CPF descriptor is not required. */
//  ble_gatts_attr_md_t*        p_user_desc_md;           /**< Attribute metadata for the User Description descriptor, or NULL for default values. */
//  ble_gatts_attr_md_t*        p_cccd_md;                /**< Attribute metadata for the Client Characteristic Configuration Descriptor, or NULL for default values. */
//  ble_gatts_attr_md_t*        p_sccd_md;                /**< Attribute metadata for the Server Characteristic Configuration Descriptor, or NULL for default values. */
//} ble_gatts_char_md_t;

//typedef struct
//{
//  ble_uuid_t          *p_uuid;          /**< Pointer to the attribute UUID. */
//  ble_gatts_attr_md_t *p_attr_md;       /**< Pointer to the attribute metadata structure. */
//  uint16_t             init_len;        /**< Initial attribute value length in bytes. */
//  uint16_t             init_offs;       /**< Initial attribute value offset in bytes. If different from zero, the first init_offs bytes of the attribute value will be left uninitialized. */
//  uint16_t             max_len;         /**< Maximum attribute value length in bytes, see @ref BLE_GATTS_ATTR_LENS_MAX for maximum values. */
//  uint8_t*             p_value;         /**< Pointer to the attribute data. Please note that if the @ref BLE_GATTS_VLOC_USER value location is selected in the attribute metadata, this will have to point to a buffer
//                                             that remains valid through the lifetime of the attribute. This excludes usage of automatic variables that may go out of scope or any other temporary location. 
//                                             The stack may access that memory directly without the application's knowledge. For writable characteristics, this value must not be a location in flash memory.*/
//} ble_gatts_attr_t;

void CustomSevice_init(void)
{  
  uint32_t err_code=0;  
  ble_uuid_t service_uuid;  
  service_uuid.type = BLE_UUID_TYPE_BLE;  
  service_uuid.uuid = BLE_SET_SERVICE_UUID;  
    
  CustomService.setgpio_level_char1_fun=SetGPIO_Char1_Function;  
  CustomService.setgpio_level_char2_fun=SetGPIO_Char2_Function;  
  CustomService.setgpio_level_char3_fun=SetGPIO_Char3_Function;  
  CustomService.setgpio_level_char4_fun=SetGPIO_Char4_Function;  
  CustomService.setuart_parity_char5_fun=SetUAUT_Parity_Char5_Function;
  CustomService.setadv_interval_char6_fun=SetADV_Interval_Char6_Function;  
  CustomService.setble_lowpowermode_char7_fun=SetBLE_Lowpowermode_Char7_Function;
    
  err_code=sd_ble_gatts_service_add (BLE_GATTS_SRVC_TYPE_PRIMARY,&service_uuid,&CustomService.service_handle);  
    
  if(err_code!=NRF_SUCCESS)
  {
    return;
  }
  err_code= CustomeService_SetGPIO_char1_add();
            CustomeService_SetGPIO_char2_add();
            CustomeService_SetGPIO_char3_add();
            CustomeService_SetGPIO_char4_add();
      CustomeService_SetUARTPARITY_char5_add();
    CustomeService_SET_ADVINTERVAL_char6_add();
   CustomeService_SET_LOWPOWERMODE_char7_add();
  
  if(err_code!=NRF_SUCCESS)
  {
    return;
  }
}

uint32_t  CustomeService_SetGPIO_char1_add(void)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC1_SETGPIO_LEVEL;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;

    return sd_ble_gatts_characteristic_add(CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_level_char1_handlers);

}

static uint32_t  CustomeService_SetGPIO_char2_add(void)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC2_SETGPIO_LEVEL;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 1;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;

    return sd_ble_gatts_characteristic_add(CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_level_char2_handlers);

}

static uint32_t  CustomeService_SetGPIO_char3_add(void)
{
 ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC3_SETGPIO_LEVEL;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add( CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_level_char3_handlers);


}

static uint32_t  CustomeService_SetGPIO_char4_add(void)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC4_SETGPIO_LEVEL;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add( CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_level_char4_handlers);


}

static uint32_t  CustomeService_SetUARTPARITY_char5_add(void)
{
 ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC5_SETUART_PARITY;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  =sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   =sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add( CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setuart_parity_char5_handlers);


}

static uint32_t  CustomeService_SET_ADVINTERVAL_char6_add(void)
{
 ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC6_SET_ADVINTERVAL;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   =sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add( CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_setadvinterval_char6_handlers);


}

static uint32_t  CustomeService_SET_LOWPOWERMODE_char7_add(void)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;


    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;

    char_md.char_props.notify =0;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

	ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = BLE_UUID_CHARACTERISTIC7_SET_LOWPOWERMODE;   

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  =sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   =sizeof(uint8_t);

    return sd_ble_gatts_characteristic_add(CustomService.service_handle,
                                            &char_md,
                                            &attr_char_value,
                                            &CustomService.setgpio_setlowpowermode_char7_handlers);


}

static void SetGPIO_Char1_Function(uint8_t data)
{
 uart_printf("recieved char1 data=%d\r\n",data);
}

static void SetGPIO_Char2_Function(uint8_t data)
{
 uart_printf("recieved char2 data=%d\r\n",data);
}

static void SetGPIO_Char3_Function(uint8_t data)
{
 uart_printf("recieved char3 data=%d\r\n",data);
}

static void SetGPIO_Char4_Function(uint8_t data)
{
 uart_printf("recieved char4 data=%d\r\n",data);
}

static void  SetUAUT_Parity_Char5_Function(uint8_t data)
{
 uart_printf("recieved char5 data=%d\r\n",data);
}

static void  SetADV_Interval_Char6_Function(uint8_t data)
{
 uart_printf("recieved char6 data=%d\r\n",data);
}

static void  SetBLE_Lowpowermode_Char7_Function(uint8_t data)
{
 uart_printf("recieved char7 data=%d\r\n",data);
}

void   CustomService_ble_evt_handle(ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ( (p_evt_write->handle==CustomService.setgpio_level_char1_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setgpio_level_char1_fun!=NULL)
       )
    
      {
        CustomService.setgpio_level_char1_fun(p_evt_write->data[0]);
      }    

    if ( (p_evt_write->handle==CustomService.setgpio_level_char2_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setgpio_level_char2_fun!=NULL)
       )
    
      {
        CustomService.setgpio_level_char2_fun(p_evt_write->data[0]);
      }     

     if ( (p_evt_write->handle==CustomService.setgpio_level_char3_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setgpio_level_char3_fun!=NULL)
       )
    
      {
        CustomService.setgpio_level_char3_fun(p_evt_write->data[0]);
      }    
      
      
     if ( (p_evt_write->handle==CustomService.setgpio_level_char4_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setgpio_level_char4_fun!=NULL)
       )
    
      {
        CustomService.setgpio_level_char4_fun(p_evt_write->data[0]);
      }    
      
     if ( (p_evt_write->handle==CustomService.setuart_parity_char5_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setuart_parity_char5_fun!=NULL)
       )
    
      {
        CustomService.setuart_parity_char5_fun(p_evt_write->data[0]);
      }    
      
     if ( (p_evt_write->handle==CustomService.setgpio_setadvinterval_char6_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setadv_interval_char6_fun!=NULL)
       )
    
      {
        CustomService.setadv_interval_char6_fun(p_evt_write->data[0]);
      }    
      
     if ( (p_evt_write->handle==CustomService.setgpio_setlowpowermode_char7_handlers.value_handle)&&
            (p_evt_write->len == 1)&&(CustomService.setble_lowpowermode_char7_fun!=NULL)
       )
    
      {
        CustomService.setble_lowpowermode_char7_fun(p_evt_write->data[0]);
      }
      
}




