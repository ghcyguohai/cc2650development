

#ifndef BLE_UART_CENTRAL_PROFILE_H

#define BLE_UART_CENTRAL_PROFILE_H

#include "ble_nus_c.h"


#define SCAN_INTERVAL               0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(30, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                  /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(500, UNIT_10_MS)    /**< Determines supervision time-out in units of 10 milliseconds. */



/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
    uint8_t     * p_data;    /**< Pointer to data. */
    uint16_t      data_len;  /**< Length of data. */
} data_t;

void db_discovery_init(void);

void nus_c_init(void);

void scan_start(void);

void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt);

void on_ble_central_evt(const ble_evt_t * const p_ble_evt);

uint32_t adv_report_parse(uint8_t type, data_t * p_advdata, data_t * p_typedata);

#endif
