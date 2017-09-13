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
#include "app_error.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"

#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"

#include "softdevice_handler.h"

#include "ble_advdata.h"

#include "ble_nus_c.h"
#include "ble_uart_central_profile.h"


#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                             /**< UART RX buffer size. */
                                                                
#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_UNKNOWN      /**< UUID type for the Nordic UART Service (vendor specific). */
                                                                
#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */
                                                                
#define APPL_LOG                app_trace_log                   /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */
                                                                                                                 
#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE	            4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

extern ble_db_discovery_t           m_ble_db_discovery;                           
extern ble_nus_c_t                  m_ble_nus_c;


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */ 
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                while (ble_nus_c_string_send(&m_ble_nus_c, data_array, index) != NRF_SUCCESS)			
                {
                    // repeat until sent
                }
                index = 0;
            }
            break;
        /**@snippet [Handling data from UART] */ 
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

 
/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = 11,
        .tx_pin_no    = 9,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}

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
    uart_init();
    ble_central_mode();
    APPL_LOG("Scan started\r\n");
    for (;;)
    {
        power_manage();
    }
}
