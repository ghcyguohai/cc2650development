
#include "ble_uart_central_profile.h"

#include "sdk_errors.h"
#include "app_error.h"
#include "string.h"
#include "ble_db_discovery.h"
#include "softdevice_handler.h"
#include "nrf_log.h"
#include "app_uart.h"
#include "pca10028.h"
#include "app_trace.h"
#include "Ble_Passthrough_Handle.h"


/**< Determines supervision time-out in units of 10 milliseconds. */
#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< */
#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(500, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UART_TX_BUF_SIZE        512                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        512                             /**< UART RX buffer size. */ 

ble_db_discovery_t              m_ble_db_discovery;                           
ble_nus_c_t                     m_ble_nus_c;

static  uint8_t                 central_tx_count=0;
static  uint8_t                 ble_central_con_flag=0;
static const uint8_t peripheral_addr[6]={0x7A,0xF6,0xAF,0x92,0x77,0xE5};


/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params = 
  {
    .active      = SCAN_ACTIVE,
    .selective   = SCAN_SELECTIVE,
    .p_whitelist = NULL,
    .interval    = SCAN_INTERVAL,
    .window      = SCAN_WINDOW,
    .timeout     = SCAN_TIMEOUT
  };


/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,   // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,   // Maximum connection
    0,                                   // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT        // Supervision time-out
  };

void ble_central_mode(void)
{
    uint32_t err_code;
    uart_init();
    ble_stack_init();
    db_discovery_init();
    err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code); 
    nus_c_init();
    scan_start();
}
  
  
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = false;
#ifdef S120
    ble_enable_params.gap_enable_params.role              = BLE_GAP_ROLE_CENTRAL;
#endif

    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

 
/**
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)   
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;
    
    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;
    
    err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init_t);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t err_code;

    sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_params);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}


static void central_on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;	
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;
           
           
          //  if (is_uuid_present(&m_nus_uuid, p_adv_report))
            if(memcmp(peripheral_addr,p_adv_report->peer_addr.addr,6)==0)
            {
              
                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param);
                
                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    app_trace_log("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                                     p_adv_report->peer_addr.addr[0],
                                     p_adv_report->peer_addr.addr[1],
                                     p_adv_report->peer_addr.addr[2],
                                     p_adv_report->peer_addr.addr[3],
                                     p_adv_report->peer_addr.addr[4],
                                     p_adv_report->peer_addr.addr[5]
                                  );
                }
            }
            break;
        }
        
        case BLE_GAP_EVT_CONNECTED:
            
            m_ble_nus_c.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            app_trace_log("target handle=%x\r\n",m_ble_nus_c.conn_handle);
            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            ble_central_con_flag=1;
            app_trace_log("BLE_GAP_EVT_CONNECTED\r\n");
            break;
    
        case BLE_GAP_EVT_DISCONNECTED:
        
             ble_central_con_flag=0;
             app_trace_log("BLE_GAP_EVT_DISCONNECTED\r\n");
        break;


        case BLE_EVT_TX_COMPLETE:
                //central_tx_count=0;
                CounterReset_UartTimer();
                ble_central_send();
        break;
        
        
        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                app_trace_log("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                app_trace_log("[APPL]: Connection Request timed out.\r\n");
            }
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
    
        default:
            break;
    }
}

static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;

  
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_FOUND_NUS_TX_CHARACTERISTIC:
            app_trace_log("The device has the device TX characteristic\r\n");
            break;
        
        case BLE_NUS_C_EVT_FOUND_NUS_RX_CHARACTERISTIC:
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            app_trace_log("The device has the device RX characteristic\r\n");
            break;
        
        case BLE_NUS_C_EVT_NUS_RX_EVT:
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                while(app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
            }
            break;
        
        case BLE_NUS_C_EVT_DISCONNECTED:
            app_trace_log("NUS device disconnected\r\n");
            scan_start();
            break;
    }
}

  
/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *  been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    central_on_ble_evt(p_ble_evt);  
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
}
  
/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
         .rx_pin_no    = 11,
         .tx_pin_no    = 9,
        .rts_pin_no   = 10,
        .cts_pin_no   = 8,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        central_uart_event_handle,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
static void central_uart_event_handle(app_uart_evt_t * p_event)
{
   uint8_t temp=0;
   // uart_printf("x%",p_event->evt_type);
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:    
           
            app_uart_get(&temp); 
            Ble_TxFifo_DataPush(&temp,1);
            if(Ble_TxFifo_Datalen()<512)
            {
               ClearCounter_UartTimer();
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

void ble_central_send(void)
{
     uint16_t len=0;
     uint32_t err_code=0;   
     static   uint8_t ble_txbuf[20]={0};
     do
     {
        len=Ble_TxFifo_Datalen();
        if(len>20)
            len=20;
        sd_ble_tx_buffer_count_get(&central_tx_count); 
        if(len>0)
        {
            sd_ble_tx_buffer_count_get(&central_tx_count); 
            Ble_TxFifo_DataPop(ble_txbuf,len);
            err_code=ble_nus_c_string_send(&m_ble_nus_c,ble_txbuf,len);
            central_tx_count++;
        }
    }while((err_code==NRF_SUCCESS)&&(len>0)&&(central_tx_count<7));
}

void ble_central_connection_state(void)
{



}

uint8_t ble_central_con_status(void)
{
    return ble_central_con_flag;
}
