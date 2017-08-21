/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "app_util_platform.h"
#include "ble.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "at.h"
#include "nrf_drv_timer.h"
#include "Ble_Passthrough_Handle.h"
#include "ble_configservice.h"
#include "nrf_uart.h"
#include "Ble_FirmwareInfo.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "Neuron_Nordic"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_BLE                  /**< UUID type for the Nordic UART Service (vendor specific). */

//广播间隔和超时时间
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

//RTC和软定时器数目
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

////////连接最小最大间隔20~75ms
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)  
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)  
//若没有数据，可以跳过多少个间隔
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
//连接后超时时间
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
//蓝牙连接后，多久之后使用上面参数，下一个参数，最大次数xxx
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01    
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF  
                                        /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

       ble_nus_t                        m_nus;
       
static dm_application_instance_t        m_app_handle;                              /**< Application identifier allocated by device manager */
static ble_dfu_t                        m_dfus;          /**< Structure to identify the Nordic UART Service. */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

//广播UUID
static  ble_uuid_t                        m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
const   nrf_drv_timer_t                   TIMER_UART = NRF_DRV_TIMER_INSTANCE(1);
extern  BLE_CFG_FIRMWAREINFO              ble_cfg_firmwareinfo;


static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void advertising_stop(void);
static void app_context_load(dm_handle_t const * p_handle);
static void reset_prepare(void);
static void device_manager_init(bool erase_bonds);
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result);
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 * GAP参数初始化
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions许可 and appearance.表现
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    //安全模式结构体设置为mode1，level1
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
 
	ble_gap_addr_t p_addr;
	//int dev_len = strlen(DEVICE_NAME);
    int devname_len=0;
    
	char dev_name[31]={0};
	//strcpy(dev_name,DEVICE_NAME);
	memcpy(dev_name,ble_cfg_firmwareinfo.lename_ssid,12);
	err_code = sd_ble_gap_address_get(&p_addr);
    
    memcpy(&dev_name[12],(uint8_t*)p_addr.addr,6);
    devname_len=sprintf(&dev_name[12],"%x%x%x%x%x%x",p_addr.addr[0],p_addr.addr[1],p_addr.addr[2],p_addr.addr[3],p_addr.addr[4],p_addr.addr[5]);

     
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) dev_name,
                                          12+devname_len);

    APP_ERROR_CHECK(err_code);

    //将gap全部初始化为0，感觉多此一举
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    //写入gap参数
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 * 处理串口数据，将从Service收到的数据发送到uart，由Service 事件调用
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 * 
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        //发送串口数据，若失败则一直发送
        while(app_uart_put(p_data[i]) != NRF_SUCCESS); 
    }
  // Test_Pin_Invert();
}
/**@snippet [Handling the data received over BLE] */

static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

   // nrf_delay_ms(500);
}
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);


    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }


    return NRF_SUCCESS;
}
/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}

static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t err_code;
    static uint32_t context_data;
    dm_application_context_t context;

    context.len = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void DFU_Service_Init(void)
{
    ble_dfu_init_t dfus_init;
    uint32_t err_code;
    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));
    dfus_init.evt_handler = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.revision = DFU_REVISION;
    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);
    dfu_app_reset_prepare_set(reset_prepare);
   // dfu_app_dm_appl_instance_set(m_app_handle);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    //数据接收处理函数指针
    nus_init.data_handler = nus_data_handler;
    
    //写入参数
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 * 连接参数事件，每次连接都会调用？？？
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 * 
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 * 连接参数初始化处理
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;    //每次连接调用的处理函数
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 * 广播事件处理
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    //只处理两种广播
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
             err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
             APP_ERROR_CHECK(err_code);
        
            break;
        case BLE_ADV_EVT_IDLE:
           // sleep_mode_enter();
            break;
        default: 
            break;
    }
}


/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t    err_code=NRF_SUCCESS;
    //协议栈事件
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            Ble_ConnectStatus_SetHigh();
            LEDS_ON(BSP_LED_2_MASK);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            LEDS_OFF(BSP_LED_2_MASK);
            Ble_ConnectStatus_SetLow();
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

//系统事件调度
static void sys_evt_dispatch(uint32_t evt_id)
{
	pstorage_sys_event_handler(evt_id);
}
/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 * 协议栈事件调度
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    dm_ble_evt_handler(p_ble_evt);
    Ble_tx_complete_handler(p_ble_evt);
}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.订阅BLE事件处理函数
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
	
	//订阅系统事件处理函数
	err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
	APP_ERROR_CHECK(err_code);
}

/**
 * time事件处理函数
 **/
extern uint8_t ble_tx_compelete_state;
extern uint8_t startup_send_flag;
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    uint8_t len,temp[32]={0};
    static uint16_t  time_counter=0;
	switch(event_type)
	{
        case NRF_TIMER_EVENT_COMPARE0:      	 
          
            time_counter++;
        
            if(time_counter>100)
            {
                time_counter=0;
                 LEDS_INVERT(BSP_LED_0_MASK);
            }
               
             if(Check_ATcmd_Pin_Level()==0)  // 0: cmd_mode
             {
                 len=Ble_TxFifo_Datalen();
                 if(len>32)
                 {
                  len=32;
                 }
                 Ble_TxFifo_DataPop(temp,len);
                 Ble_ATcmd_Parser(temp,len);
             }
             else
             {
                 Ble_Timout_Send();
             }
             
        break;
        
        default:
            //Do nothing.
        break;		
	}
}

void Uart_Poll_Send(void)
{
    uint8_t temp=0;
    uint16_t len=0;
    uint16_t i=0;

    len=Ble_TxFifo_Datalen();
    for(i=0;i<len;i++)
    {
        Ble_TxFifo_DataPop(&temp,1);
        while(app_uart_put(temp)!=NRF_SUCCESS); 
    }
}

/**
 * 定时器，收到UART最后一个数据n ms后未收到下一个数据，表示数据接收结束
 * 形参，毫秒，建议2字节以上，即波特率的20倍以上
 * 9600=104us,115200=8.7us
 * 目前开放设置波特率为9600/19200/38400/115200。超时固定最低波特率的20倍，即2ms(104us*20)
 * 因不支持9600以下波特率，所以该值不需要改
 **/
//const nrf_drv_timer_t TIMER_UART = NRF_DRV_TIMER_INSTANCE(1);
void time_init(uint32_t time_ms)
{
	uint32_t err_code = NRF_SUCCESS;
   
    err_code = nrf_drv_timer_init(&TIMER_UART, NULL, timer_event_handler);
    APP_ERROR_CHECK(err_code);	
	
    //将定时时间转成定时器的tick值
    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_UART, time_ms);
    
    //初始化定时时间，CLEAR清除计数器
    nrf_drv_timer_extended_compare(
         &TIMER_UART, NRF_TIMER_CC_CHANNEL0, time_ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    //nrf_drv_timer_compare(
      //   &TIMER_UART, NRF_TIMER_CC_CHANNEL0, time_ticks, true);    
    nrf_drv_timer_enable(&TIMER_UART);	
}


/**@brief   Function for handling app_uart events.
 * 
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 * 这里发送是ASCII，不是16进制，需要大改
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t temp=0;
   // uart_printf("x%",p_event->evt_type);
    switch (p_event->evt_type)
    {
        //数据准备完毕，取数据
        case APP_UART_DATA_READY:    
            
           //Test_Pin_Invert();
            app_uart_get(&temp); 
            Ble_TxFifo_DataPush(&temp,1);
            if(Ble_TxFifo_Datalen()<512)
            {
                nrf_drv_timer_clear(&TIMER_UART);
            }
            
            break;
        
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
        
        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_HIGH,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */
/**@brief Function for initializing buttons and leds.
 * 初始化IO口
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void board_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_ON(LEDS_MASK);	
    LEDS_OFF(BSP_LED_2_MASK);
}

/**@brief Application main function.
 */
int main(void)
{ 
    uint32_t err_code;	
    bool erase_bonds=false;
    
    Ble_Fifo_Init();  
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
	board_init();
 
    ATcmd_Pin_Configuration();
    Ble_ConnectStatusPin_Configuration();
    time_init(10);
    FirmwareInfo_Default();
    device_manager_init(erase_bonds);
    ble_stack_init();

    gap_params_init();
    DFU_Service_Init();
    CustomSevice_init();
    services_init();
    advertising_init();
    conn_params_init();
    sd_ble_gap_tx_power_set(4);  // set txpower is 4db
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
   

    for (;;)
    {  
       // uart_printf("mcu is running \r\n");
        if(Check_ATcmd_Pin_Level())
        {
          LEDS_ON(BSP_LED_3_MASK);
        }
        else
        {
          LEDS_OFF(BSP_LED_3_MASK);
        }          
    }
}
