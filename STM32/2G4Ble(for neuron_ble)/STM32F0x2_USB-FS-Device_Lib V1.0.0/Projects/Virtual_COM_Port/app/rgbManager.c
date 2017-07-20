/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "hw_config.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
//#include "dcm.h"

#include "rgbManager.h"

void RGB_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  // RGB1 :	PA15: 	TIM2 CH1 ETR
  //			PB3:	TIM2 CH2
  //			PB4:	TIM3 CH1

  // RGB2 :	PB5:	TIM3 CH2
  //			PB6:	TIM4 CH1
  //			PB7:	TIM4 CH2

  // MOTO :	PB0:	TIM3 CH3
  //			PB1:	TIM3 CH4
  //			PB10:	TIM2 CH3
  //			PB11:	TIM2 CH4

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); //ʹ��GPIO�����AFIO���ù���ģ��ʱ��

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	//ʹ��A15����JTAG��ͻ,��ֹJTAG������SWD
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//����������� n
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO

  // TIM2
  TIM_TimeBaseStructure.TIM_Period = 255 - 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = 12; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

  //��ʼ��TIMPWMģʽ
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//0��ƽ

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  // ch3 and 4 is init for motor pwm output
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_Cmd(TIM2, ENABLE);

  // TIM3
  TIM_TimeBaseStructure.TIM_Period = 255 - 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = 12; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

  //��ʼ��TIMPWMģʽ
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//0��ƽ

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  // ch3 and 4 is init for motor pwm output
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_Cmd(TIM3, ENABLE);

  // TIM4
  TIM_TimeBaseStructure.TIM_Period = 255 - 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler = 12; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

  //��ʼ��TIMPWMģʽ
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//0��ƽ

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_Cmd(TIM4, ENABLE);

}


void rgbMngInit(void)
{
  RGB_Init();
  rgbShow();
}

// RGB1 :	PA15: 	TIM2 CH1 ETR 	Green
//			PB3:	TIM2 CH2		Red
//			PB4:	TIM3 CH1		Blue

// RGB2 :	PB5:	TIM3 CH2		Green
//			PB6:	TIM4 CH1		Red
//			PB7:	TIM4 CH2		Blue
void rgbShow(void)
{
  if(robot.param.mode == MODE_NORMAL)
  {
    TIM_SetCompare2(TIM3, robot.rgb.r);
    TIM_SetCompare1(TIM4, robot.rgb.g);
    TIM_SetCompare2(TIM4, robot.rgb.b);
  }
}


void rgbMngClose(void)
{
  //TIM_SetCompare1(TIM2, 0);
  //TIM_SetCompare2(TIM2, 0);
  //TIM_SetCompare1(TIM3, 0);
  TIM_SetCompare2(TIM3, 0);
  TIM_SetCompare1(TIM4, 0);
  TIM_SetCompare2(TIM4, 0);
}


