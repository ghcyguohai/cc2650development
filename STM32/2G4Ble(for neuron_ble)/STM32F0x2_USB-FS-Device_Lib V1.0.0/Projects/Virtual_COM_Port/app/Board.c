


#include "Board.h"
//#include "hw_config.h"
#include "stm32f0xx.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "Broadcom_Ble.h"

void USB_pullup_pin_cfg(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_5);
}

void USB_pullup_High(void)
{
  GPIO_SetBits(GPIOB,GPIO_Pin_5);
}

 void   Borad_Start_Init   (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; // rgb
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;   // this io is used to read usb connect event
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//∏°ø’ ‰»Î
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
    
   
  USB_pullup_pin_cfg();  
  Ble_IO_Init();
}

void Board_Led_off(uint8_t num)
{
	if(num==0)
	{
	  GPIO_SetBits(GPIOB,GPIO_Pin_7);
	}
	else if(num==1)
	{
	  GPIO_SetBits(GPIOB,GPIO_Pin_8);
	}
	else if(num==2)
	{
	 GPIO_SetBits(GPIOB,GPIO_Pin_9);
	}
}

void Board_Led_on(uint8_t num)
{
    if(num==0)
    {
        GPIO_ResetBits(GPIOB,GPIO_Pin_7);
    }
    else if (num==1)
    {
        GPIO_ResetBits(GPIOB,GPIO_Pin_8);
    }
    else if (num==2)
    {
    GPIO_ResetBits(GPIOB,GPIO_Pin_9);
    }
}

void Board_Led_Toggle(uint8_t led_num)
{
  static uint8_t R,G,B;

  if(led_num==0)
  {
    if(R==0)
    {
      R=1;
      GPIO_SetBits(GPIOB,GPIO_Pin_7);
    }
   else
   {
     R=0;
     GPIO_ResetBits(GPIOB,GPIO_Pin_7);
   }

  }

  if(led_num==1)
  {
    if(G==0)
    {
      G=1;
      GPIO_SetBits(GPIOB,GPIO_Pin_8);
    }
   else
   {
     G=0;
     GPIO_ResetBits(GPIOB,GPIO_Pin_8);
   }

  }


  if (led_num==2)
  {
     if(B==0)
    {
      B=1;
      GPIO_SetBits(GPIOB,GPIO_Pin_9);
    }
   else
   {
     B=0;
     GPIO_ResetBits(GPIOB,GPIO_Pin_9);
   }
  }
}

void LED_RGB__Flash_Set_Status(LED_FLASH_STATE* led_flash_state )
{
//#define RED_NUM    0u
//#define GREEN_NUM  1u
//#define BLUE_NUM   2u

//  Board_Led_Toggle(led_type);
//  vTaskDelay( 500/portTICK_RATE_MS );
}
