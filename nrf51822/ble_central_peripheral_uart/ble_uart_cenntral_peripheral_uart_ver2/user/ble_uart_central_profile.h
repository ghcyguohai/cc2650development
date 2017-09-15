

#ifndef BLE_UART_CENTRAL_PROFILE_H

#define BLE_UART_CENTRAL_PROFILE_H

#include "ble_nus_c.h"
#include "app_uart.h"


         

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t     * p_data;    /**< Pointer to data. */
    uint16_t      data_len;  /**< Length of data. */
} data_t;

void ble_central_mode(void);

static void ble_stack_init(void);

static void db_discovery_init(void);

static void nus_c_init(void);

static void scan_start(void);

static void central_on_ble_evt(ble_evt_t * p_ble_evt);

static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt);

static void ble_evt_dispatch(ble_evt_t * p_ble_evt);

static void uart_init(void);

static void central_uart_event_handle(app_uart_evt_t * p_event);

       void ble_central_send(void);

       void ble_central_connection_state(void);
       
    uint8_t ble_central_con_status(void);

#endif
