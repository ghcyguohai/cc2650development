#ifndef _AT_C_
#define _AT_C_


static  int StringFind(const char *pSrc, const char *pDst);

void    ATcmd_Pin_Configuration(void);
uint8_t Check_ATcmd_Pin_Level  (void);
void    Ble_ConnectStatusPin_Configuration(void);
void    Ble_ConnectStatus_SetHigh(void);
void    Ble_ConnectStatus_SetLow(void);
void    Test_Pin_Invert(void);
void    Ble_ATcmd_Parser(uint8_t *pbuf,uint16_t len);




void    Ble_ATcmd_LENAME_Handler(uint8_t* pbuf, uint8_t len);
void    Ble_ATcmd_LENAMEReply_Handler(uint8_t* pbuf, uint8_t len);
void    Ble_ATcmd_LEMAC_Handler(uint8_t* pbuf, uint8_t len);
void    Ble_ATcmd_QuerySoftwareVerReply_Handler(uint8_t* pbuf, uint8_t len);
void    Ble_ATcmd_SerURATE_Handler(uint8_t* pbuf, uint8_t len);
void    Ble_ATcmd_QueyURATE_Handler(uint8_t* pbuf, uint8_t len);



void    at_cmd_start(uint8_t *data);
void    hex_to_ascii(uint8_t dat, char *h, char *l);
void    uart_printf(char* fmt,...);

#endif
