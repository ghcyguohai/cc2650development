
#include "AppTask_Uart3.h"

#include "stm32f0xx.h"
#include "Ring_Buf.h"
#include "Board.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "Old_Protocol.h"
#include "AppTask_Start.h"
#include "serialManager.h"

#define  BIT(x)  (1u<<x)


#define  UART3_RX_FLAG BIT(0)
#define  UART3_TX_FLAG BIT(1)
#define  UART3_RX_BUFSIZE       256
#define  UART3_DMA_RX_BUFSIZE   256
#define  UART3_UNPACK_CACHE_SIZE 256


       RING_BUF_DEF_STRUCT  UART3_Recieve_Ringbuf;
static uint8_t              UART3_Rx_Buf[UART3_RX_BUFSIZE];
static uint8_t              UART3_DMA_Rx_Buf[UART3_DMA_RX_BUFSIZE];
static EventGroupHandle_t   Uart3_EventGroup_Handle = NULL;

       PARSER_HANDLE        Uart3_Protocol_Unpack_Handle;
static      uint8_t         Uart3_Unpack_Cache[UART3_UNPACK_CACHE_SIZE];

extern PASSTHROUGH_HANDLE   Passthrough_Handle;


void AppTask_Uart3_Create(void)
{
  Uart3_EventGroup_Handle=xEventGroupCreate();
	
	if(Uart3_EventGroup_Handle!=NULL)
	{
	   xTaskCreate(AppTask_Uart3,"UartTask Create",128,NULL,0,NULL);	
	}
}

static void AppTask_Uart3(void *pvParameters)
{
	  EventBits_t   Uart_Event_Bits=0;
	
	  AppTask_Uart3_Init();

	while(1)
	{
       Uart_Event_Bits =xEventGroupWaitBits(
                                              Uart3_EventGroup_Handle,
                                              UART3_RX_FLAG|UART3_TX_FLAG,
                                              pdTRUE, 
                                              pdFALSE,
                                              portMAX_DELAY
                                            );  // portMAX_DELAY: indicate: the task will wait forever
       if(Uart_Event_Bits&UART3_RX_FLAG)
        {	

             USB_Printf("Recieved data from UART3 \r\n");           
			 Uart3_Protocol_Unpack_Handle.passthrough_handle->uart_online_starus.uart3_online_timerout_cnt=10000;
             Uart_PassThrough(&Uart3_Protocol_Unpack_Handle,RingBuf_Count(Uart3_Protocol_Unpack_Handle.ring_buf));	// pass the unused data		 
        }				 
       if(Uart_Event_Bits&UART3_TX_FLAG)
        {				 
        }
   }
}


 void  AppTask_Uart3_Init(void)
{
    RingBuf_Init(&UART3_Recieve_Ringbuf,UART3_Rx_Buf,UART3_RX_BUFSIZE);
    
	UART3_Recieve_Ringbuf.uart=USART3;
	Uart3_Protocol_Unpack_Handle.parser_cache=Uart3_Unpack_Cache;
	Uart3_Protocol_Unpack_Handle.parser_cache_maxsize=UART3_UNPACK_CACHE_SIZE;
	Uart3_Protocol_Unpack_Handle.ring_buf=&UART3_Recieve_Ringbuf;
	Uart3_Protocol_Unpack_Handle.step=0;
	Uart3_Protocol_Unpack_Handle.parser_cache_index=0;
	Uart3_Protocol_Unpack_Handle.passthrough_handle=&Passthrough_Handle;
	
    Uart3_Config();
	Uart3_Rx_Dma_Config();
}

 static void  Uart3_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_4);  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_4);    
    
  USART_DeInit(USART3); 
  //USART1_TX   PB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽输出
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;  
  GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PA2

  //USART1_RX	  PB.11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//浮空输入
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
  USART_Init(USART3, &USART_InitStructure); 

  NVIC_InitStructure.NVIC_IRQChannel = USART3_4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4 ; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
  NVIC_Init(&NVIC_InitStructure);	

  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);// idle int

}

static  void Uart3_Rx_Dma_Config(void)
{
  DMA_InitTypeDef   DMA_InitStruct;
  NVIC_InitTypeDef  NVIC_Initstructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	/*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->RDR);
  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStruct.DMA_MemoryBaseAddr =(uint32_t)UART3_DMA_Rx_Buf;
  /* Initialize the DMA_DIR member */
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct.DMA_BufferSize = UART3_DMA_RX_BUFSIZE;
  /* Initialize the DMA_PeripheralInc member */
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  /* Initialize the DMA_MemoryInc member */
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  /* Initialize the DMA_Mode member */
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  /* Initialize the DMA_Priority member */
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
  /* Initialize the DMA_M2M member */
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6,&DMA_InitStruct);
  DMA_RemapConfig(DMA1, DMA1_CH6_USART3_RX);	
	
	NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Initstructure.NVIC_IRQChannel = DMA1_Channel4_5_6_7_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelPriority = 2 ;
	
	NVIC_Init(&NVIC_Initstructure);	
	
	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC|DMA_IT_HT,ENABLE);
	DMA_Cmd(DMA1_Channel6,ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

 void  Uart3_DMA_isr(void)
 {
   Uart3_Dma_RX_ISR(&UART3_Recieve_Ringbuf);
 }


 static void Uart3_Dma_RX_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    

        if(DMA_GetFlagStatus(DMA1_FLAG_HT6)!=RESET)
        {
            DMA_ClearITPendingBit(DMA1_FLAG_HT6);
            RingBuf_Write(Uart_Recieve_Ringbuf,UART3_DMA_Rx_Buf,UART3_DMA_RX_BUFSIZE/2);
            xEventGroupSetBitsFromISR(
                                     Uart3_EventGroup_Handle,
                                     UART3_RX_FLAG,
                                     &xHigherPriorityTaskWoken );  // pdTURE : interrupt exit.the schedule will start
        }
        
        if(DMA_GetFlagStatus(DMA1_FLAG_TC6)!=RESET)
        {
            DMA_ClearITPendingBit(DMA1_FLAG_TC6);
            RingBuf_Write(Uart_Recieve_Ringbuf,&UART3_DMA_Rx_Buf[UART3_DMA_RX_BUFSIZE/2],UART3_DMA_RX_BUFSIZE/2);
            xEventGroupSetBitsFromISR(
                                     Uart3_EventGroup_Handle,
                                     UART3_RX_FLAG,
                                     &xHigherPriorityTaskWoken );  // pdTURE : interrupt exit.the schedule will start
        }
       
}

static void  Uart3_IDLE_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t dma_transfer_pos=0;

    dma_transfer_pos=UART3_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(DMA1_Channel6);
	
	DMA_Cmd(DMA1_Channel6, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel6,UART3_DMA_RX_BUFSIZE);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	
	if(dma_transfer_pos<(UART3_DMA_RX_BUFSIZE/2))
	{
        RingBuf_Write(Uart_Recieve_Ringbuf,UART3_DMA_Rx_Buf,dma_transfer_pos);
	}
	
	if(dma_transfer_pos>(UART3_DMA_RX_BUFSIZE/2))
	{
	 	dma_transfer_pos=dma_transfer_pos-(UART3_DMA_RX_BUFSIZE/2);
        RingBuf_Write(Uart_Recieve_Ringbuf,&UART3_DMA_Rx_Buf[UART3_DMA_RX_BUFSIZE/2],dma_transfer_pos);
	}
   xEventGroupSetBitsFromISR(
                                 Uart3_EventGroup_Handle,
                                 UART3_RX_FLAG,
                                 &xHigherPriorityTaskWoken 
                             );  // pdTURE : interrupt exit.the schedule will start
}


//void DMA1_Channel4_5_6_7_IRQHandler(void)
//{
//    Uart3_Dma_RX_ISR(&UART3_Recieve_Ringbuf);
//}

void USART3_4_IRQHandler(void)
{
 if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //
 {
      USART_ClearITPendingBit(USART3,USART_IT_IDLE);
      USART_ReceiveData( USART3 ); // Clear IDLE interrupt flag bit
	  Uart3_IDLE_ISR(&UART3_Recieve_Ringbuf);
 }
}
