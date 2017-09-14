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


                                                                                                                         
#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */
                                                                
#define APPL_LOG                app_trace_log                   /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */
                                                                                                                 
#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE	            4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

#define BLE_ROLE_MASTER_MODE     0      


extern ble_db_discovery_t           m_ble_db_discovery;                           
extern ble_nus_c_t                  m_ble_nus_c;


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

int main(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    #if BLE_ROLE_MASTER_MODE
        ble_central_mode();
        APPL_LOG("Master mode started\r\n");
    #else
        ble_periph_mode();
        APPL_LOG("Slave mode started\r\n");
    #endif 
    
   
    for (;;)
    {
       // power_manage();
    }
}
