#include <string.h>
#include <stdio.h>
#include "can_func.h"
#include "Printf.h"
#include "stm32f4xx_can.h"
#include "timer_func.h"
#include "can_bus.h"
#include "parking.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

vu8 Can1TransmitMailbox;
vu8 Can1TransmitStatus;
vu8 Can1ReceiveStatus = 0;
CanRxMsg RxMessageArr[RMALen];
CanRxMsg GRxMessage;
vu8	 RMARPtr = 0;
vu8	 RMAWPtr = 0;
vu8 CAN_IT_FMP0_F = 0;

xQueueHandle xCan1RxDataQue = NULL;
xQueueHandle xRxCMDQue = NULL;
xSemaphoreHandle xMutexAccessDevInfo;


void Can1_InterruptConfig(void);
void GPIO_Configuration(void);

int CAN1_Config(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	//u32 i = 0;
  	//u8 TransmitMailbox;
	uint8_t RetStatus;
	//RCC_ClocksTypeDef RCC_ClocksStatus;
	


	//RCC_GetClocksFreq(&RCC_ClocksStatus);
	//printf("SYSCLK_Frequency = %d  HCLK_Frequency = %d PCLK1_Frequency = %d PCLK2_Frequency = %d\r\n",
	//RCC_ClocksStatus.SYSCLK_Frequency, RCC_ClocksStatus.HCLK_Frequency, RCC_ClocksStatus.PCLK1_Frequency, RCC_ClocksStatus.PCLK2_Frequency);

	xCan1RxDataQue = xQueueCreate( 20, sizeof(CanRxMsg));
	xRxCMDQue = xQueueCreate( 64, sizeof(u8) );
	xMutexAccessDevInfo	= xSemaphoreCreateMutex();
	

	/* CAN register init */
	
  	CAN_DeInit(CAN1);
//	RCC_AHB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
	RCC->APB1ENR |= ((1<<25));//使能CAN1时钟
	GPIO_Configuration();
	CAN_StructInit(&CAN_InitStructure);

	Can1_InterruptConfig(); 

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;			//MCR-TTCM  时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=DISABLE;			//MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=DISABLE;		    //MCR-AWUM  自动唤醒模式
	//CAN_InitStructure.CAN_NART=ENABLE;                           //MCR-NART  禁止报文自动重传          0-自动重传   1-报文只传一次
	CAN_InitStructure.CAN_NART=ENABLE;			//MCR-NART  禁止报文自动重传          0-自动重传   1-报文只传一次
	CAN_InitStructure.CAN_RFLM=DISABLE;			//MCR-RFLM  接收FIFO 锁定模式  0-溢出时新报文会覆盖原有报文  1-溢出时，新报文丢弃
	CAN_InitStructure.CAN_TXFP=ENABLE;			//MCR-TXFP  发送FIFO优先级 0-优先级取决于报文标示符 1-优先级取决于发送请求的顺序
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;	//CAN_Mode_Normal		   CAN_Mode_LoopBack  CAN_Mode_Silent_LoopBack
	//125k bandrate		   pclk 42m
	/*
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_8tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler=24;
	*/
	/*
	//125k bandrate		pclk 42m
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_16tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
	CAN_InitStructure.CAN_Prescaler=14;
	*/

	//50k bandrate		 pclk 42m
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_16tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_7tq;
	CAN_InitStructure.CAN_Prescaler=35;

	RetStatus = CAN_Init(CAN1, &CAN_InitStructure);
	if(RetStatus == CAN_InitStatus_Failed)
	{
		printf("CAN1_InitStatus_Failed\r\n");
		return -1;
	}
	else
		printf("CAN1 initial as CAN_Mode_Normal success\r\n");
	
	
	
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	return 0; /* Test Passed */
}

void Can1_InterruptConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;


//#ifdef  VECT_TAB_RAM  
//  /* Set the Vector Table base location at 0x20000000 */ 
//  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
//#else  /* VECT_TAB_FLASH  */
//  /* Set the Vector Table base location at 0x08000000 */ 
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
//#endif

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN1, CAN_IT_TME, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	CAN_ITConfig(CAN1, CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1, ENABLE);

//	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	//CAN_ITConfig(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK, ENABLE);

	 //CAN1->IER =0xffffffff;
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
   	RCC->AHB2ENR |= ((1<<1) | (1<<3) | (1<<0));//使能GPIOB、D时钟 
//  /* Configure PC.06, PC.07, PC.08 and PC.09 as Output push-pull */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//  /* Configure CAN pin: RX */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//  
//  /* Configure CAN pin: TX */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);                                //CAN IO时钟
	GPIO_DeInit(GPIOB); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                                                            //RX引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                                  //输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                                                  //设置为推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                                          
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //GPIO_PuPd_DOWN  GPIO_PuPd_UP                                              //设置上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                                                            //TX引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                                   //设置复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                                                   //设置为推挽
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                    //其他未置引脚悬空
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);                                //启动IO复用功能
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

//	RCC->AHB1ENR |= ((1<<1) | (1<<3));//使能GPIOB、D时钟  
//  GPIOB->AFR[0] |= 0x00900000;      //AF9  
//  GPIOB->AFR[1] |= 0x00900009;  
//  GPIOD->AFR[0] |= 0x00000090;  
//      
//  GPIOB->MODER &= 0xF3FCF3FF; //第二功能  
//  GPIOB->MODER |= 0x08020800;  
//  GPIOD->MODER &= 0xFFFFFFF3;   
//  GPIOD->MODER |= 0x00000008;  
//    
//  GPIOB->OSPEEDR &= 0xF3FCF3FF; //50M  
//  GPIOB->OSPEEDR |= 0x08020800;  
//  GPIOD->OSPEEDR &= 0xFFFFFFF3;   
//  GPIOD->OSPEEDR |= 0x00000008;  
//    
//  GPIOB->PUPDR &= 0xF3FCF3FF;   //上拉  
//  GPIOB->PUPDR |= 0x04010400;  
//  GPIOD->PUPDR &= 0xFFFFFFF3;   
//  GPIOD->PUPDR |= 0x00000004;   

}

void can1Send(void * pvParameters)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=0x0;
	TxMessage.IDE=CAN_Id_Extended;	  //CAN_Id_Standard 	   CAN_Id_Extended
	TxMessage.RTR=CAN_RTR_Remote;
	TxMessage.DLC=0;


	for(;;)
	{

		

		

				dest_add++;
				//dest_add = 1;
				if(dest_add == 64)
				{
					//dbg = 1;
					dest_add = 1;
				}
				TxMessage.ExtId=dest_add;
				
				//Can1TransmitStatus = 0xff;
				xSemaphoreTake(xMutexAccessDevInfo, portMAX_DELAY);	
				memset(DevInfo[dest_add], 0x0, 64);
				
				Can1TransmitMailbox=CAN_Transmit(CAN1, &TxMessage);

				vTaskDelay(100);
				xSemaphoreGive( xMutexAccessDevInfo );

		
	}

}

void can1Rec(void * pvParameters)
{
	CanRxMsg canRxMsg;

	for(;;)
	{
		xQueueReceive(xCan1RxDataQue, &canRxMsg,  portMAX_DELAY);
			{

				xSemaphoreTake(xMutexAccessDevInfo, portMAX_DELAY);
				memcpy(DevInfo[canRxMsg.StdId]+canRxMsg.ExtId*8, canRxMsg.Data, canRxMsg.DLC);
				xSemaphoreGive( xMutexAccessDevInfo );

			}
				
	}

}

#define REC_IDLE 			0
#define REC_TYPE 			1
#define REC_ID      		2
#define REC_REGION_ID 		3
#define REC_PARA1_LBS 		4
#define REC_PARA1_MBS 		5
#define REC_PARA2_LBS 		6
#define REC_PARA2_MBS 		7
#define REC_PARA3_LBS 		8
#define REC_PARA3_MBS 		9
#define REC_CHECK 			10

u8 recStartFlag = 0;
u8   recStatus = REC_IDLE;

void doCommand(void * pvParameters)
{
	u8 rec_buf, rec_index;
	u8 carifo_index, carifo_subindex;
	u32 dev_num;
	park_header spark_header;
	u8 *prec = (u8 *)&spark_header;
	for(;;)
	{
		xQueueReceive(xRxCMDQue, &rec_buf,  portMAX_DELAY);

			USART_SendData(USART1, rec_buf);

//		 	if(rec_buf = '$')
//			{
//				recStartFlag = 1;
//				rec_index = 0;
//			}
//
//			if(recStartFlag == 1)
//			{
//				memcpy(prec+rec_index, &rec_buf, 1);
//			   if((++rec_index) == 11)
//			   {
//			   		recStartFlag = 0;
//					printf("flag = %d\r\n", spark_header.flag);
//				   	printf("type = %d\r\n", spark_header.type);
//					printf("id = %d\r\n", spark_header.id);
//					printf("region_id = %d\r\n", spark_header.region_id);
//					printf("param1 = %d\r\n", spark_header.param1);
//					printf("param2 = %d\r\n", spark_header.param2);
//					printf("param3 = %d\r\n", spark_header.param3);	 
//					printf("check = %d\r\n", spark_header.check);
//			   }
//			   
//
//			}

		 
		
		

//		if(rec_buf == 'S')
//		{
//			xSemaphoreTake(xMutexAccessDevInfo, portMAX_DELAY);
//			dev_num = 0	;
//		   for(carifo_index = 1; carifo_index < 64; carifo_index++)
//		   {
//				for(carifo_subindex = 0; carifo_subindex < 64; carifo_subindex++)
//					{
//						if(((DevInfo[carifo_index][carifo_subindex]) & 0x0f)== 0xf)
//							{
//								printf("DevInf[%d][%d]\r\n",carifo_index, carifo_subindex);
//								dev_num++;
//							}
//
//					}
//				
//					
//		   }
//		   xSemaphoreGive( xMutexAccessDevInfo );
//		  printf("########Device num  = %d\r\n",dev_num);
//
//		}

	}

}




void setLeds(void * pvParameters)
{
	

	for(;;)
	{
		
	}

}


