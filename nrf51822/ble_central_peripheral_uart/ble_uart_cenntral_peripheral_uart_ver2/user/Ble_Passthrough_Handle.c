

#include "Ble_Passthrough_Handle.h"
#include "at.h"
#include "app_error.h"
#include "ble_uart_central_profile.h"
#include "ble_uart_peripheral_profile.h"
#include "app_trace.h"


#define   BLE_PASSTHROUGH_BUF_SIZE  1024u


BLE_CONNECT_STATUS Ble_Con_Status=BLE_CONNECT_DISCONNECTED;

uint8_t ble_con_flag=0;


static uint8_t ble_passthrough_rxbuf[BLE_PASSTHROUGH_BUF_SIZE]={0};
static uint8_t ble_passthrough_txbuf[BLE_PASSTHROUGH_BUF_SIZE]={0};

static app_fifo_t ble_rxfifo;
static app_fifo_t ble_txfifo;


const   nrf_drv_timer_t                   TIMER_UART = NRF_DRV_TIMER_INSTANCE(1);


void Ble_Fifo_Init(void)
{
    app_fifo_init( &ble_rxfifo, ble_passthrough_rxbuf, BLE_PASSTHROUGH_BUF_SIZE);
    app_fifo_init( &ble_txfifo, ble_passthrough_txbuf, BLE_PASSTHROUGH_BUF_SIZE);
    app_fifo_flush(&ble_rxfifo);
    app_fifo_flush(&ble_txfifo);
}

void Ble_TxFifo_DataPush(uint8_t* pbuf,uint32_t len)
{
    uint32_t templengh=0;
    templengh=len;

    app_fifo_write(&ble_txfifo,pbuf,&templengh);
}

void Ble_TxFifo_ReadIndexBackfoward(uint32_t len)
{
    ble_txfifo.read_pos=(ble_txfifo.read_pos-len)&ble_txfifo.buf_size_mask;
}

void Ble_RxFifo_DataPush(uint8_t* pbuf,uint32_t len)
{
    uint32_t templengh=0;
    templengh=len;
    app_fifo_write(&ble_rxfifo,pbuf,&templengh);
}

void Ble_TxFifo_DataPop(uint8_t* pbuf,uint32_t len)
{
    uint32_t templengh=0;
    templengh=len;
    app_fifo_read(&ble_txfifo,pbuf,&templengh);
}

void Ble_RxFifo_DataPop(uint8_t* pbuf,uint32_t len)
{
    uint32_t templengh=0;
    templengh=len;
    app_fifo_read(&ble_rxfifo,pbuf,&templengh);
}

uint16_t Ble_TxFifo_Datalen(void)
{
    return fifo_length(&ble_txfifo);
}

uint16_t Ble_RxFifo_Datalen(void)
{
    return fifo_length(&ble_rxfifo);
}




void time_init(uint32_t time_ms)
{
	uint32_t err_code = NRF_SUCCESS;
   
    err_code = nrf_drv_timer_init(&TIMER_UART, NULL, timer_event_handler);
    APP_ERROR_CHECK(err_code);	
    
    uint32_t time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_UART, time_ms);
    
    nrf_drv_timer_extended_compare(&TIMER_UART, NRF_TIMER_CC_CHANNEL0, time_ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);  
    nrf_drv_timer_enable(&TIMER_UART);	
}

void ClearCounter_UartTimer(void)
{
   nrf_drv_timer_clear(&TIMER_UART);
}

void CounterReset_UartTimer(void)
{
   nrf_drv_timer_clear(&TIMER_UART);
   nrf_drv_timer_enable(&TIMER_UART);
}

static  void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    uint8_t len,temp[32]={0};
    static uint16_t  time_counter=0;
	switch(event_type)
	{
        case NRF_TIMER_EVENT_COMPARE0:      	 
          
            time_counter++;
        
            if(time_counter>100)
            {
                time_counter=0;             
            }
               
             if((ble_periph_con_status()==0)&&(ble_central_con_status()==0))  // 0: cmd_mode
             {
                 len=Ble_TxFifo_Datalen();
                 if(len>32)
                 {
                  len=32;
                 }
                 Ble_TxFifo_DataPop(temp,len);
                 Ble_ATcmd_Parser(temp,len);
             }
             else
             {
                 if(ble_central_con_status()==1)
                 {
                    ble_central_send();
                 }
               
                 if(ble_periph_con_status()==1)
                 {
                    ble_periph_send();
                 }
                
             }
             
        break;
        
        default:
            //Do nothing.
        break;		
	}
}




