

#ifndef  BLE_UART_PERIPHERAL_H
#define  BLE_UART_PERIPHERAL_H

#include "ble_nus.h"
#include "app_error.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

void services_init(void);

void advertising_init(void);

void on_adv_evt(ble_adv_evt_t ble_adv_evt);

void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);


#endif

