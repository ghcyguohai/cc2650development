

#ifndef  BROADCOM_BLE_H

#define  BROADCOM_BLE_H

#include "stdint.h"
#include "Ring_Buf.h"

typedef __packed struct 
{
     uint8_t pair_flag;   // 1:  paired, 0: not paired  
     uint8_t ble_mode;
     uint8_t ble_selfmac[12];
     uint8_t ble_targetmac[12];
}BLE_CFGINFO;


void    Ble_Info_Init(void);
void    Ble_Info_Read(BLE_CFGINFO* dst);
void    Ble_Info_Write(BLE_CFGINFO* src);
void    Ble_IO_Init(void);
static  void Ble_CmdPin_Set(uint8_t mode);
void    Ble_ConnectStatus_Monitor(void);
void    Ble_ATCmdAutoLink(void);
uint8_t Ble_ATCmdPassthrough_Send(void);
void    Ble_ATCmd_Tick(void);
uint8_t Ble_ATCmd_ReadMac(uint8_t* p_buf,uint8_t len);
uint8_t Ble_ATCmd_ReadTargetMac(uint8_t* p_buf,uint8_t len);
static uint8_t Ble_ATCmdDisconnecct(void);
static uint8_t Ble_ATCmdPassthrough_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf);
static uint8_t Ble_ATCmdDisconnecct_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf);
static uint8_t Ble_ATCmd_ReadMac_Res_Check(RING_BUF_DEF_STRUCT* src_ring_buf);
static int StringFind(const char *pSrc, const char *pDst);

#endif
