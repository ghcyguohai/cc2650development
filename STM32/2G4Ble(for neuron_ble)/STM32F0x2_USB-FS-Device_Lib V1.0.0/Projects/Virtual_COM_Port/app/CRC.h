


uint8_t Get_Crc8(uint8_t *puchMsg, uint8_t usDataLen) ;
uint16_t Get_Crc16(uint8_t *puchMsg,uint16_t usDataLen);
uint8_t Check_Crc8(uint8_t* src,uint8_t len);
uint8_t Check_Crc16(uint8_t* src,uint16_t len);
void     Attach_Crc8(uint8_t* src,uint8_t len);
void  Attach_Crc16(uint8_t* src,uint16_t len);