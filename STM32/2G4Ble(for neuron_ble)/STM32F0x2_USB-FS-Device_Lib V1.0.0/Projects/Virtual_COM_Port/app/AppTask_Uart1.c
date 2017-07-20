

#include "Ring_Buf.h"
#include "Board.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "Old_Protocol.h"

#include "AppTask_Uart1.h"
#include "AppTask_Uart2.h"
#include "AppTask_Uart3.h"

#include "AppTask_Start.h"
#include "hw_config.h"
#include "serialManager.h"

#define  BIT(x)  (1u<<x)

#define  UART1_RX_FLAG BIT(0)
#define  UART1_TX_FLAG BIT(1)
#define  USB_RX_FLAG_BIT  BIT(2)

#define  UART2_RX_FLAG BIT(3)
#define  UART2_TX_FLAG BIT(4)
#define  UART3_RX_FLAG BIT(5)
#define  UART3_TX_FLAG BIT(6)


#define  UART1_RX_BUFSIZE  256
#define  UART1_DMA_RX_BUFSIZE 256
#define  UART1_UNPACK_CACHE_SIZE 256


RING_BUF_DEF_STRUCT         UART1_Recieve_Ringbuf;

static uint8_t              UART1_Rx_Buf[UART1_RX_BUFSIZE];
static uint8_t              UART1_DMA_Rx_Buf[UART1_DMA_RX_BUFSIZE];
static EventGroupHandle_t   Uart1_EventGroup_Handle = NULL;

       PARSER_HANDLE   Uart1_Protocol_Unpack_Handle;
static      uint8_t    Uart1_Unpack_Cache[UART1_UNPACK_CACHE_SIZE];


extern PARSER_HANDLE Uart2_Protocol_Unpack_Handle;
extern PARSER_HANDLE Uart3_Protocol_Unpack_Handle;

extern PASSTHROUGH_HANDLE   Passthrough_Handle;

extern RING_BUF_DEF_STRUCT  USB_Recieve_Ringbuf;
extern uint8_t              USB_Rx_Buf[USB_RX_BUFSIZE];



void AppTask_Uart1_Create(void)
{
  Uart1_EventGroup_Handle=xEventGroupCreate();
	if(Uart1_EventGroup_Handle!=NULL)
	{		
	   xTaskCreate(AppTask_Uart1,"UartTask Create",128,NULL,0,NULL);	
	}
}

static void AppTask_Uart1(void *pvParameters)
{
	  EventBits_t   Uart_Event_Bits=0;
      uint8_t       temp_data;
    
      RingBuf_Init(&USB_Recieve_Ringbuf,USB_Rx_Buf,USB_RX_BUFSIZE);
      USB_Recieve_Ringbuf.uart=NULL;  // is the com is USB ,value 0x00 to UART
	  AppTask_Uart1_Init();
      
	while(1)
	{
        Uart_Event_Bits =xEventGroupWaitBits(
                                          Uart1_EventGroup_Handle,
                                          UART1_RX_FLAG|UART1_TX_FLAG|USB_RX_FLAG_BIT,
                                          pdTRUE, 
                                          pdFALSE,
                                          portMAX_DELAY
                                        );  // portMAX_DELAY: indicate: the task will wait forever   	
           if(Uart_Event_Bits&UART1_RX_FLAG)
           {
              USB_Printf("Recieved data from UART1 \r\n");  
              Uart1_Protocol_Unpack_Handle.passthrough_handle->uart_online_starus.uart1_online_singal=1;
              Uart1_Protocol_Unpack_Handle.passthrough_handle->uart_online_starus.uart1_online_timerout_cnt=10000;           
              Cmd_Unpack(&Uart1_Protocol_Unpack_Handle);
           }
           if(Uart_Event_Bits&UART1_TX_FLAG)
           {
           }
                   
           if(Uart_Event_Bits&USB_RX_FLAG_BIT)
           {
            while(RingBuf_Count(&USB_Recieve_Ringbuf))
                {             
                  RingBuf_Read(&USB_Recieve_Ringbuf,1,&temp_data);
                  Uart_Send(Uart2_Protocol_Unpack_Handle.ring_buf->uart,&temp_data,1);  // this code is just for bluetooth
                } 
           }  
   }
}

void  AppTask_Uart1_Init(void)
{
    RingBuf_Init(&UART1_Recieve_Ringbuf,UART1_Rx_Buf,UART1_RX_BUFSIZE);
	UART1_Recieve_Ringbuf.uart=USART1;
	Uart1_Protocol_Unpack_Handle.parser_cache=Uart1_Unpack_Cache;
	Uart1_Protocol_Unpack_Handle.parser_cache_maxsize=UART1_UNPACK_CACHE_SIZE;
	Uart1_Protocol_Unpack_Handle.ring_buf=&UART1_Recieve_Ringbuf;
	Uart1_Protocol_Unpack_Handle.step=0;
	Uart1_Protocol_Unpack_Handle.parser_cache_index=0;
	Uart1_Protocol_Unpack_Handle.passthrough_handle=&Passthrough_Handle;
    Uart1_Config();
	Uart1_Rx_Dma_Config();
}

static void  Uart1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);	//使能USART1，GPIOA时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);  
    
  USART_DeInit(USART1);  //复位串口1
  //USART1_TX   PA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽输出
  GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType= GPIO_OType_PP;  
    
  GPIO_Init(GPIOA, &GPIO_InitStructure); //

  //USART1_RX	  PA.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
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
  USART_Init(USART1, &USART_InitStructure); //初始化串口

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 4 ; //抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);// idle int
}

static void Uart1_Rx_Dma_Config(void)
{
  DMA_InitTypeDef   DMA_InitStruct;
  NVIC_InitTypeDef  NVIC_Initstructure;
	
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	/*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStruct.DMA_MemoryBaseAddr =(uint32_t)UART1_DMA_Rx_Buf;
  /* Initialize the DMA_DIR member */
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct.DMA_BufferSize = UART1_DMA_RX_BUFSIZE;
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
  DMA_Init(DMA1_Channel3,&DMA_InitStruct);
  DMA_RemapConfig(DMA1, DMA1_CH3_USART1_RX);
	
	NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Initstructure.NVIC_IRQChannel = DMA1_Channel2_3_IRQn;
    NVIC_Initstructure.NVIC_IRQChannelPriority = 2 ;
		
	NVIC_Init(&NVIC_Initstructure);	
	
	DMA_ITConfig(DMA1_Channel3,DMA_IT_TC|DMA_IT_HT,ENABLE);
	DMA_Cmd(DMA1_Channel3,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

static void Uart1_Dma_RX_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	if(DMA_GetFlagStatus(DMA1_FLAG_HT3)!=RESET)
	{
	 DMA_ClearITPendingBit(DMA1_FLAG_HT3);
	 RingBuf_Write(Uart_Recieve_Ringbuf,UART1_DMA_Rx_Buf,UART1_DMA_RX_BUFSIZE/2);
     xEventGroupSetBitsFromISR(
                                 Uart1_EventGroup_Handle,
                                 UART1_RX_FLAG,
                                 &xHigherPriorityTaskWoken
                               );  // pdTURE : interrupt exit.the schedule will start
	}
	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3)!=RESET)
	{
	 DMA_ClearITPendingBit(DMA1_FLAG_TC3);
	 RingBuf_Write(Uart_Recieve_Ringbuf,&UART1_DMA_Rx_Buf[UART1_DMA_RX_BUFSIZE/2],UART1_DMA_RX_BUFSIZE/2);
      xEventGroupSetBitsFromISR(
                                Uart1_EventGroup_Handle,
                                UART1_RX_FLAG,
                                &xHigherPriorityTaskWoken
                                );  // pdTURE : interrupt exit.the schedule will start   
	}
	


}

static void  Uart1_IDLE_ISR(RING_BUF_DEF_STRUCT*  Uart_Recieve_Ringbuf)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t   dma_transfer_pos=0;

    dma_transfer_pos=UART1_DMA_RX_BUFSIZE-DMA_GetCurrDataCounter(DMA1_Channel3);
	
	DMA_Cmd(DMA1_Channel3, DISABLE);
	DMA_SetCurrDataCounter(DMA1_Channel3,UART1_DMA_RX_BUFSIZE);
	DMA_Cmd(DMA1_Channel3, ENABLE);
	
	if(dma_transfer_pos<(UART1_DMA_RX_BUFSIZE/2))
	{
		 RingBuf_Write(Uart_Recieve_Ringbuf,UART1_DMA_Rx_Buf,dma_transfer_pos);
	}
	
	if(dma_transfer_pos>(UART1_DMA_RX_BUFSIZE/2))
	{
        dma_transfer_pos=dma_transfer_pos-(UART1_DMA_RX_BUFSIZE/2);
        RingBuf_Write(Uart_Recieve_Ringbuf,&UART1_DMA_Rx_Buf[UART1_DMA_RX_BUFSIZE/2],dma_transfer_pos);
	}
   xEventGroupSetBitsFromISR(
                             Uart1_EventGroup_Handle,
                             UART1_RX_FLAG,
                             &xHigherPriorityTaskWoken );  // pdTURE : interrupt exit.the schedule will start
}

 void DMA1_Channel2_3_IRQHandler(void)
{
 Uart1_Dma_RX_ISR(&UART1_Recieve_Ringbuf);
}

void USART1_IRQHandler(void)
{
 if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //
 {
    USART_ClearITPendingBit(USART1,USART_IT_IDLE);
    USART_ReceiveData( USART1 ); // Clear IDLE interrupt flag bit
    Uart1_IDLE_ISR(&UART1_Recieve_Ringbuf);
 }
}

void USB_User_Recieve_ISR(uint8_t* pbuf,uint16_t len)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
    RingBuf_Write(&USB_Recieve_Ringbuf,pbuf,len);  // this code is added by user
    
     xEventGroupSetBitsFromISR(
                                 Uart1_EventGroup_Handle,
                                 USB_RX_FLAG_BIT,
                                 &xHigherPriorityTaskWoken
                              );  // pdTURE : interrupt exit.the schedule will start
    
   
}

