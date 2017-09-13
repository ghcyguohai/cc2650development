
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



ble_db_discovery_t                m_ble_db_discovery;                           
ble_nus_c_t                        m_ble_nus_c;
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
            app_trace_log("Connected to target\r\n");
          //  err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            
            m_ble_nus_c.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            app_trace_log("target handle=%x\r\n",m_ble_nus_c.conn_handle);
            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
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
  
  

