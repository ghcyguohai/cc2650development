

#include "Ble_Passthrough_Handle.h"
#include "ble_configservice.h"
#include "ble_nus.h"
#include "at.h"


#define   BLE_PASSTHROUGH_BUF_SIZE  1024u




extern ble_nus_t  m_nus; 
extern const nrf_drv_timer_t TIMER_UART; 

BLE_CONNECT_STATUS Ble_Con_Status=BLE_CONNECT_DISCONNECTED;

static uint8_t ble_con_flag=0;
static uint8_t tx_count=0;

static uint8_t ble_passthrough_rxbuf[BLE_PASSTHROUGH_BUF_SIZE]={0};
static uint8_t ble_passthrough_txbuf[BLE_PASSTHROUGH_BUF_SIZE]={0};

static app_fifo_t ble_rxfifo;
static app_fifo_t ble_txfifo;

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


void ClearCounter_UartTimer(void)
{
   nrf_drv_timer_clear(&TIMER_UART);
}

void Ble_Timout_Send(void)
{
    if(ble_con_flag==1)
    {
        Ble_Send();
        //Test_Pin_Invert();
        //uart_printf("timer is running\r\n");
    } 
}

void Ble_Send(void)
{
     uint16_t len=0;
     uint32_t err_code=0;   
     static   uint8_t ble_txbuf[20]={0};
     do
     {
        len=Ble_TxFifo_Datalen();
        if(len>20)
        len=20;
        if(len>0)
        {
            sd_ble_tx_buffer_count_get(&tx_count); 
            //uart_printf("tx_count=%d\r\n",tx_count);
            Ble_TxFifo_DataPop(ble_txbuf,len);
            err_code=ble_nus_string_send(&m_nus,ble_txbuf,len);
            tx_count++;
            uart_printf("err_code=%x\r\n",err_code);
        }
    }while((err_code==NRF_SUCCESS)&&(len>0)&&(tx_count<7));
}

void Ble_tx_complete_handler(ble_evt_t * p_ble_evt)
{
   // uart_printf("ble_evt=%x \r\n",p_ble_evt->header.evt_id);
    
    switch(p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
                uart_printf("ble connected ok\r\n");
                Ble_Con_Status=BLE_CONNECT_CONNECTED;      
        break;
        
        case BLE_GAP_EVT_DISCONNECTED:
                uart_printf("ble disconnected ok\r\n");
                Ble_Con_Status=BLE_CONNECT_DISCONNECTED;   // disconnect
                ble_con_flag=2;

        break;
        
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
                uart_printf("ble connn para upadate successfully\r\n");
                Ble_Con_Status=BLE_CONNECT_CONPARA_UPDATE;   // disconnect
                ble_con_flag=1;
        break;
        
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            ble_con_flag=1;
        break;
        
        case BLE_GATTS_EVT_WRITE:
            ble_con_flag=1;
            CustomService_ble_evt_handle(p_ble_evt);
   
        
        break;
        
        case BLE_EVT_TX_COMPLETE:
              //uart_printf("ble transmit ok\r\n"); 
                tx_count=0;
                CounterReset_UartTimer();
                Ble_Con_Status=BLE_CONNECT_TRANSMIT_COMPELE;
                Ble_Send();
               // Test_Pin_Invert();
        break;
        
        default:
        break;
    }
    uart_printf("evtid=%x\r\n",p_ble_evt->header.evt_id);
}

void CounterReset_UartTimer(void)
{
   nrf_drv_timer_clear(&TIMER_UART);
   nrf_drv_timer_enable(&TIMER_UART);
}
void HardFault_Handler(void)
{
    uart_printf("hard fault\r\n");
}
