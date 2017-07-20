



#ifndef BOARD_H
#define BOARD_H

#include "stdint.h"

#define RED_NUM    0u
#define GREEN_NUM  1u
#define BLUE_NUM   2u


typedef struct 
{
 uint8_t red_led_freq;
 uint8_t red_trig_flag;
 uint8_t green_led_freq;
 uint8_t green_trig_flag;
 uint8_t blue_led_freq;
 uint8_t blue_trig_flag;	

}LED_FLASH_STATE;


void   USB_pullup_pin_cfg(void);
void   USB_pullup_High(void);
void   Borad_Start_Init   (void);
void   Board_Led_off(uint8_t num);
void   Board_Led_on(uint8_t num);
void   Board_Led_Toggle(uint8_t led_num);
#endif 

