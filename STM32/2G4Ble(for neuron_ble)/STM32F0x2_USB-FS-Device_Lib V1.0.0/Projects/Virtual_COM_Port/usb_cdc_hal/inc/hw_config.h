

#ifndef  HW_CONFIG_H

#include "stdint.h"



void USB_CDC_Init(void);
void USB_CDC_Send(uint8_t* src,uint16_t len);
void USB_CDC_int_tx_isr(void);
void USB_CDC_Recieve(uint8_t* src,uint16_t len);

#endif
