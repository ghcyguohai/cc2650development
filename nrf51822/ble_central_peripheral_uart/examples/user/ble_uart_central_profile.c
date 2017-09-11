
#include "ble_uart_central_profile.h"

#include "sdk_errors.h"
#include "app_error.h"
#include "string.h"
#include "ble_db_discovery.h"
#include "nrf_log.h"
#include "app_uart.h"
#include "pca10028.h"

ble_db_discovery_t                m_ble_db_discovery;                           
static const uint8_t peripheral_addr[6]={0x7A,0xF6,0xAF,0x92,0x77,0xE5};
ble_nus_c_t                  m_ble_nus_c;

/** @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_param =
{
    0,              // Active scanning not set.
    0,              // Selective scanning not set.
    0,           // No whitelist provided.
    SCAN_INTERVAL,
    SCAN_WINDOW,
    0x0000          // No timeout.
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    0,
    (uint16_t)SUPERVISION_TIMEOUT
};

/**
 * @brief Database discovery initialization.
 */
void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client.
 */
void nus_c_init(void)   
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;
    
    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;
    
    err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init_t);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning.
 */
void scan_start(void)
{
    ret_code_t err_code;

    sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}


void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;

    NRF_LOG_PRINTF("Recieved data\r\n");
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_FOUND_NUS_TX_CHARACTERISTIC:
            NRF_LOG_PRINTF("The device has the device TX characteristic\r\n");
            break;
        
        case BLE_NUS_C_EVT_FOUND_NUS_RX_CHARACTERISTIC:
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_PRINTF("The device has the device RX characteristic\r\n");
            break;
        
        case BLE_NUS_C_EVT_NUS_RX_EVT:
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                while(app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
            }
            NRF_LOG_PRINTF("recieved data from slave\r\n");
            break;
        
        case BLE_NUS_C_EVT_DISCONNECTED:
            NRF_LOG_PRINTF("NUS device disconnected\r\n");
           // scan_start();
            break;
    }
}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 * parses scanning reports, initiating a connection attempt to peripherals when a target UUID
 * is found, and manages connection parameter update requests. Additionally, it updates the status
 * of LEDs used to report central applications activity.
 *
 * @note        Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *              should be dispatched to the target application before invoking this function.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void on_ble_central_evt(const ble_evt_t * const p_ble_evt)
{
    // For readability.
    const ble_gap_evt_t   * const p_gap_evt = &p_ble_evt->evt.gap_evt;

 //   NRF_LOG_PRINTF("gap evet=%d\r\n",p_ble_evt->header.evt_id);
    switch (p_ble_evt->header.evt_id)
    {
        /** Upon connection, check which peripheral has connected (HR or RSC), initiate DB
         *  discovery, update LEDs status and resume scanning if necessary. */
        case BLE_GAP_EVT_CONNECTED:
            {
                uint32_t err_code;
                // For readability.
               // const ble_gap_addr_t * const peer_addr = &p_gap_evt->params.connected.peer_addr;
                NRF_LOG_PRINTF("connect tartget device successully\r\n");
                m_ble_nus_c.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                NRF_LOG_PRINTF("connect tartget handle=%d\r\n",m_ble_nus_c.conn_handle);
                err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
                if(err_code==NRF_SUCCESS)
                {
                    NRF_LOG_PRINTF("ble_db_discovery_start successfully\r\n");
                }
               // APP_ERROR_CHECK(err_code);
            } 
        break; // BLE_GAP_EVT_CONNECTED

        /** Upon disconnection, reset the connection handle of the peer which disconnected, update
         * the LEDs status and start scanning again. */
        case BLE_GAP_EVT_DISCONNECTED:
            {
               NRF_LOG_PRINTF("recieved gap disconnect evt\r\n");
               m_ble_nus_c.conn_handle=BLE_CONN_HANDLE_INVALID;
               scan_start();  
            }
        break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_ADV_REPORT:
            {
                uint32_t err_code;
                data_t   adv_data;
                data_t   type_data;
                uint8_t  i=0;
                // For readibility.
                const ble_gap_addr_t  * const peer_addr = &p_gap_evt->params.adv_report.peer_addr;
                
                if(memcmp(peripheral_addr,peer_addr->addr,6)==0)
                    {    
                        NRF_LOG_PRINTF("target addr_type =%x\r\n",peer_addr->addr_type);
                        NRF_LOG_PRINTF("target addr_addr =%x%x%x%x%x%x\r\n",peer_addr->addr[0],peer_addr->addr[1],peer_addr->addr[2],peer_addr->addr[3],peer_addr->addr[4],peer_addr->addr[5]);
                        for(i=0;i<p_gap_evt->params.adv_report.dlen;i++)
                        {
                            NRF_LOG_PRINTF("%c",p_gap_evt->params.adv_report.data[i]);
                        }  
                            NRF_LOG_PRINTF("\r\n");
                        adv_data.p_data     = (uint8_t *)p_gap_evt->params.adv_report.data;
                        adv_data.data_len   = p_gap_evt->params.adv_report.dlen;
  
                        err_code = adv_report_parse(BLE_GAP_AD_TYPE_FLAGS,
                                                    &adv_data,
                                                    &type_data);
                        if(err_code==NRF_SUCCESS)
                        {
//                            err_code = sd_ble_gap_scan_stop();
//                            if(err_code!=NRF_SUCCESS)
//                            {
//                                NRF_LOG_PRINTF("scan stop failed\r\n");
//                            }
                            NRF_LOG_PRINTF("found the target peripheral\r\n");
                        }
                        
                        err_code = sd_ble_gap_connect(peer_addr,
                                                      &m_scan_param,
                                                      &m_connection_param);
                        
                       if(err_code==NRF_SUCCESS)
                       {
                            NRF_LOG_PRINTF("send connection req successfully \r\n");
                       }
                    } 
                      // APP_ERROR_CHECK(err_code);                    // Initialize advertisement report for parsing.
            } 
        break; // BLE_GAP_ADV_REPORT
         
            
        case BLE_GAP_EVT_TIMEOUT:
            {
                // We have not specified a timeout for scanning, so only connection attemps can timeout.
                if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
                {
                   NRF_LOG_PRINTF("ble gap evt timeout\r\n");
                }
            } 
        break; // BLE_GAP_EVT_TIMEOUT

            
        case    BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            
                NRF_LOG_PRINTF("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
        
        break;
            
        
        case    BLE_GAP_EVT_CONN_PARAM_UPDATE:
            
                 NRF_LOG_PRINTF("BLE_GAP_EVT_CONN_PARAM_UPDATE\r\n");
        break;

        
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            {
                // Accept parameters requested by peer.
                ret_code_t err_code;
                err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                           &p_gap_evt->params.conn_param_update_request.conn_params);
                
                if(err_code==NRF_SUCCESS)
                {
                    NRF_LOG_PRINTF("para update request handle successfully\r\n");
                }
                else
                {
                     NRF_LOG_PRINTF("para update request handle failed\r\n");
                }
               // APP_ERROR_CHECK(err_code);
            } 
        break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];
        
        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index+2];
            p_typedata->data_len = field_length-1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}
