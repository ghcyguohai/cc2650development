#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"

#include "nrf.h"
#include "ble_db_discovery.h"

#include "app_timer.h"
#include "app_trace.h"
#include "app_util.h"
#include "bsp.h"
#include "ble_uart_central_profile.h"
#include "ble_uart_peripheral_profile.h"
#include "Ble_Passthrough_Handle.h"

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */
                                                                
#define APPL_LOG                app_trace_log                   /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */                                                                                                               
#define BLE_ROLE_MASTER_MODE    0     



int main(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    Ble_Fifo_Init();
    time_init(10);
    
    #if BLE_ROLE_MASTER_MODE
        ble_central_mode();
        APPL_LOG("Master mode started\r\n");
    #else
        ble_periph_mode();
        APPL_LOG("Slave mode started\r\n");
    #endif 
    
    for (;;)
    {
        
    }
}
