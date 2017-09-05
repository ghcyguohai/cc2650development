

#ifndef BLE_UART_CENTRAL_PROFILE_H

#define BLE_UART_CENTRAL_PROFILE_H

#include "ble_nus_c.h"


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
