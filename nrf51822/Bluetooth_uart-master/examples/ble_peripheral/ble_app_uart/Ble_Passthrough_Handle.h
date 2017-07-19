
#ifndef BLE_PASSTHROUGH_HANDLE_H

#define BLE_PASSTHROUGH_HANDLE_H

#include "app_fifo.h"
#include "ble.h"
#include "nrf_drv_timer.h"

typedef enum 
{
    BLE_CONNECT_CONNECTED,
    BLE_CONNECT_DISCONNECTED,
    BLE_CONNECT_CONPARA_UPDATE,
    BLE_CONNECT_TRANSMIT_COMPELE    
}BLE_CONNECT_STATUS;


void Ble_Fifo_Init(void);

void Ble_TxFifo_DataPush(uint8_t* pbuf,uint32_t len);

void Ble_TxFifo_ReadIndexBackfoward(uint32_t len);

void Ble_RxFifo_DataPush(uint8_t* pbuf,uint32_t len);

void Ble_TxFifo_DataPop(uint8_t* pbuf,uint32_t len);

void Ble_RxFifo_DataPop(uint8_t* pbuf,uint32_t len);

uint16_t Ble_TxFifo_Datalen(void);

uint16_t Ble_RxFifo_Datalen(void);

void ClearCounter_UartTimer(void);

void Ble_Timout_Send(void);

void Ble_Send(void);

void Ble_tx_complete_handler(ble_evt_t * p_ble_evt);

void CounterReset_UartTimer(void);


#endif 



