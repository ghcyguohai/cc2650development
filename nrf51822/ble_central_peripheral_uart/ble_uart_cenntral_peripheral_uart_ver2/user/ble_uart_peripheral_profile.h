

#ifndef  BLE_UART_PERIPHERAL_H
#define  BLE_UART_PERIPHERAL_H

#include "ble_nus.h"
#include "app_error.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_uart.h"

void ble_periph_mode(void);

static void gap_params_init(void);

static void  conn_params_init(void);

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);

static void conn_params_error_handler(uint32_t nrf_error);

static void ble_stack_init(void);

static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);


static void advertising_init(void);

static void services_init(void);

static void ble_evt_dispatch(ble_evt_t * p_ble_evt);

static void peripheral_on_ble_evt(ble_evt_t * p_ble_evt);

static void uart_init(void);

static void uart_event_handle(app_uart_evt_t * p_event);

#endif




