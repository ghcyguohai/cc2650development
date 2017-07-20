

#include "Read_ModuleID.h"


#define  ID1_AD_CHANNEL  ADC_Channel_5
#define  ID2_AD_CHANNEL  ADC_Channel_6

void ModuleID_ADPin_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef   ADC_InitStructure;
     
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5; // 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_OD; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);    

   ADC_DeInit(ADC1);  
   /* Initialize ADC structure */
   ADC_StructInit(&ADC_InitStructure);
  /* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
   ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
   ADC_Init(ADC1, &ADC_InitStructure);
   
 //  ADC_ChannelConfig(ADC1, ID1_AD_CHANNEL , ADC_SampleTime_239_5Cycles);
   ADC_ChannelConfig(ADC1, ID1_AD_CHANNEL , ADC_SampleTime_239_5Cycles);
    
   /* ADC Calibration */
   ADC_GetCalibrationFactor(ADC1);
   /* Enable the ADC peripheral */
   ADC_Cmd(ADC1, ENABLE);     
   /* Wait the ADRDY flag */
     /* Wait the ADRDY flag */
   while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));
    /* ADC1 regular Software Start Conv */ 
    //ADC_StartOfConversion(ADC1);
}

uint16_t  Read_ModuleID(void )
{
    
 uint16_t   ID1_ADC1ConvertedValue=0; 
 uint16_t   ID2_ADC1ConvertedValue=0;   
 uint8_t    ID1_num=0;
 uint8_t    ID2_num=0;    
 uint8_t    i=0;  
    
     for( i=0;i<10;i++)
     {   
       // ADC_StopOfConversion(ADC1); 
         
        ADC_ClearFlag(ADC1,ADC_FLAG_EOC);  
        ADC_ChannelConfig(ADC1, ID1_AD_CHANNEL , ADC_SampleTime_239_5Cycles);  
         
        ADC_StartOfConversion(ADC1);       
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        ID1_ADC1ConvertedValue+=ADC_GetConversionValue(ADC1);
     }
 
    for( i=0;i<10;i++)
     {   
       // ADC_StopOfConversion(ADC1); 
         
        ADC_ClearFlag(ADC1,ADC_FLAG_EOC);  
        ADC_ChannelConfig(ADC1, ID1_AD_CHANNEL , ADC_SampleTime_239_5Cycles);  
         
        ADC_StartOfConversion(ADC1);       
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        ID2_ADC1ConvertedValue+=ADC_GetConversionValue(ADC1);
     }

  ID1_num=ID1_ADC1ConvertedValue/(10*256);  // ID range is 0-15, ID_value_max=4096/256=16
  ID2_num=ID2_ADC1ConvertedValue/(10*256);
 
return ID1_num+(ID2_num<<8);
}
