#include "stm32f0xx.h"
#include "AppTask_Uart2.h"
#include "AppTask_Uart3.h"

#include "Ring_Buf.h"
#include "Board.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "serialManager.h"
#include "Old_Protocol.h"
#include "AppTask_Start.h"



#define  BIT(x)  (1u<<x)


#define  UART2_RX_FLAG BIT(0)
#define  UART2_TX_FLAG BIT(1)
#define  UART2_RX_BUFSIZE       256
#define  UART2_DMA_RX_BUFSIZE   256
#define  UART2_UNPACK_CACHE_SIZE 256


       RING_BUF_DEF_STRUCT  UART2_Recieve_Ringbuf;
static uint8_t              UART2_Rx_Buf[UART2_RX_BUFSIZE];
static uint8_t              UART2_DMA_Rx_Buf[UART2_DMA_RX_BUFSIZE];
static EventGroupHandle_t   Uart2_EventGroup_Handle = NULL;

           PARSER_HANDLE   Uart2_Protocol_Unpack_Handle;
static      uint8_t    Uart2_Unpack_Cache[UART2_UNPACK_CACHE_SIZE];

extern PASSTHROUGH_HANDLE   Passthrough_Handle;



void AppTask_Uart2_Create(void)
{
  Uart2_EventGroup_Handle=xEventGroupCreate();
	if(Uart2_EventGroup_Handle!=NULL)
	{
	   xTaskCreate(AppTask_Uart2,"UartTask Create",128,NULL,0,NULL);	
	}
}

static void AppTask_Uart2(void *pvParameters)
{
	  EventBits_t   Uart_Event_Bits=0;

	  AppTask_Uart2_Init();
     // uart2_printf("Uart2 start success \r\n");
        while(1)
        {
           Uart_Event_Bits =xEventGroupWaitBits(
                                                  Uart2_EventGroup_Handle,
                                                  UART2_RX_FLAG|UART2_TX_FLAG,
                                                  pdTRUE, 
                                                  pdFALSE,
                                                  portMAX_DELAY
                                                );  // portMAX_DELAY: indicate: the task will wait forever 
               if(Uart_Event_Bits&UART2_RX_FLAG)
               {
                  USB_Printf("Recieved data from UART2 \r\n");  
                  Uart2_Protocol_Unpack_Handle.passthrough_handle->uart_online_starus.uart2_online_singal  =1;
                  Uart2_Protocol_Unpack_Handle.passthrough_handle->uart_online_starus.uart2_online_timerout_cnt=10000;
                  Cmd_Unpack(&Uart2_Protocol_Unpack_Handle);
               }
               if(Uart_Event_Bits&UART2_TX_FLAG)
               {
               }
       }
}


 void  AppTask_Uart2_Init(void)
{
    RingBuf_Init(&UART2_Recieve_Ringbuf,UART2_Rx_Buf,UART2_RX_BUFSIZE);
	UART2_Recieve_Ringbuf.uart=USART2;
	Uart2_Protocol_Unpack_Handle.parser_cache=Uart2_Unpack_Cache;
	Uart2_Protocol_Unpack_Handle.parser_cache_maxsize=UART2_UNPACK_CACHE_SIZE;
	Uart2_Protocol_Unpack_Handle.ring_buf=&UART2_Recieve_Ringbuf;
	Uart2_Protocol_Unpack_Handle.step=0;
	Uart2_Protocol_Unpack_Handle.parser_cache_index=0;
	Uart2_Protocol_Unpack_Handle.passthrough_handle=&Passthrough_Handle;
    Uart2_Config();
	Uart2_Rx_Dma_Config();
}

 static void  Uart2_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);  
  USART_DeInit(USART2);  //复位串口2
  //USART1_TX   PA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽输出
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;  
  GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2

  //USART1_RX	  PA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//浮空输入
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4 ; //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);// idle int
}

static  void Uart2_Rx_Dma_Config(void)
{
  DMA_InitTypeDef   DMA_InitStruct;
  NVIC_InitTypeDef  NVIC_Initstructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	/*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->RDR);
  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStruct.DMA_MemoryBaseAddr =(uint32_t)UART2_DMA_Rx_Buf;
  /* Initialize the DMA_DIR member */
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct.DMA_BufferSize = UART2_DMA_RX_BUFSIZE;
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
  DMA_Init(DMA1_Channel5,&DMA_InitStruct);
  DMA_RemapConfig(DMA1, DMA1_CH5_USART2_RX);
	
	NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Initstructure.NVIC_IRQChannel = DMA1_Channel4_5_6_7_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelPriority = 2 ;
		
	NVIC_Init(&NVIC_Initstructure);	
	
	DMA_ITConfig(DMA1_Channel5,DMA_IT_TC|DMA_IT_HT,ENABLE);
	DMA_Cmd(DMA1_Channel5,ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART2, ENABLE);

}

static void Uart2_Dma_RX_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	

        if(DMA_GetFlagStatus(DMA1_FLAG_HT5)!=RESET)
        {
            DMA_ClearITPendingBit(DMA1_FLAG_HT5);
            RingBuf_Write(Uart_Recieve_Ringbuf,UART2_DMA_Rx_Buf,UART2_DMA_RX_BUFSIZE/2);
            xEventGroupSetBitsFromISR(
                                        Uart2_EventGroup_Handle,
                                        UART2_RX_FLAG,
                                        &xHigherPriorityTaskWoken );  // pdTURE : interrupt exit.the schedule will start
        }
        
        if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET)
        {
            DMA_ClearITPendingBit(DMA1_FLAG_TC5);
            RingBuf_Write(Uart_Recieve_Ringbuf,&UART2_DMA_Rx_Buf[UART2_DMA_RX_BUFSIZE/2],UART2_DMA_RX_BUFSIZE/2);
            xEventGroupSetBitsFromISR(
                                        Uart2_EventGroup_Handle,
                                        UART2_RX_FLAG,
                                        &xHigherPriorityTaskWoken );  // pdTURE : interrupt exit.the schedule will start
        }
        

}

static void  Uart2_IDLE_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t dma_transfer_pos=0;

    dma_transfer_pos=UART2_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(DMA1_Channel5);
	
	DMA_Cmd(DMA1_Channel5, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel5,UART2_DMA_RX_BUFSIZE);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	
	if(dma_transfer_pos<(UART2_DMA_RX_BUFSIZE/2))
	{
        RingBuf_Write(Uart_Recieve_Ringbuf,UART2_DMA_Rx_Buf,dma_transfer_pos);
	}
	
	if(dma_transfer_pos>(UART2_DMA_RX_BUFSIZE/2))
	{
        dma_transfer_pos=dma_transfer_pos-(UART2_DMA_RX_BUFSIZE/2);
        RingBuf_Write(Uart_Recieve_Ringbuf,&UART2_DMA_Rx_Buf[UART2_DMA_RX_BUFSIZE/2],dma_transfer_pos);
	}
   xEventGroupSetBitsFromISR(
                                 Uart2_EventGroup_Handle,
                                 UART2_RX_FLAG,
                                 &xHigherPriorityTaskWoken 
                            );  // pdTURE : interrupt exit.the schedule will start
}


void DMA1_Channel4_5_6_7_IRQHandler(void)
{
 Uart2_Dma_RX_ISR(&UART2_Recieve_Ringbuf);
 Uart3_DMA_isr();
}

void USART2_IRQHandler(void)
{
 if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  //
 {
    USART_ClearITPendingBit(USART2,USART_IT_IDLE);
    USART_ReceiveData( USART2 ); // Clear IDLE interrupt flag bit
    Uart2_IDLE_ISR(&UART2_Recieve_Ringbuf);
 }
}

 void  Uart2_RingBuf_Rx_Flush(void)
 {
   RingBuf_Flush(&UART2_Recieve_Ringbuf);
 }
