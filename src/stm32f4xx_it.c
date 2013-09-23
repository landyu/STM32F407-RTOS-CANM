/**
  ******************************************************************************
  * @file    stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "stm32f4x7_eth.h"

/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* lwip includes */
#include "lwip/sys.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern xSemaphoreHandle s_xSemaphore;
/* Private function prototypes -----------------------------------------------*/
extern void xPortSysTickHandler(void); 
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  printf("\n\r HardFault_Handler ");
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  printf("\n\r MemManage_Handler ");
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  printf("\n\r BusFault_Handler ");
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  printf("\n\r UsageFault_Handler ");
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  
  timer100mscouter++;
	timer500mscouter++;
	//counter++;
	 xPortSysTickHandler();
	if(timer100mscouter >=100)
		{
			timer100mscouter = 0;
			timeflag |= F100ms;
			//flag1s = 1;
//			if(!DevExisting)
//			nolot(dest_add);
//
//			SendCan = 1;
		}
	if(timer500mscouter >=500)
		{
			timer500mscouter = 0;
			timeflag |= F500ms;

			
			//flag1s = 1;
		}
}

/**
  * @brief  This function handles External line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
  {
    Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
    /* Clear interrupt pending bit */
    EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
  }
}
#if 0
/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(ETH_LINK_EXTI_LINE) != RESET)
  {
    Eth_Link_ITHandler(DP83848_PHY_ADDRESS);
    /* Clear interrupt pending bit */
    EXTI_ClearITPendingBit(ETH_LINK_EXTI_LINE);
  }
}
#endif
/**
  * @brief  This function handles ethernet DMA interrupt request.
  * @param  None
  * @retval None
  */
void ETH_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* Frame received */
  if ( ETH_GetDMAFlagStatus(ETH_DMA_FLAG_R) == SET) 
  {
    /* Give the semaphore to wakeup LwIP task */
    xSemaphoreGiveFromISR( s_xSemaphore, &xHigherPriorityTaskWoken );   
  }
	
  /* Clear the interrupt flags. */
  /* Clear the Eth DMA Rx IT pending bits */
  ETH_DMAClearITPendingBit(ETH_DMA_IT_R);
  ETH_DMAClearITPendingBit(ETH_DMA_IT_NIS);
	
  /* Switch tasks if necessary. */	
  if( xHigherPriorityTaskWoken != pdFALSE )
  {
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/



void USART1_IRQHandler(void)
{
	u8 tmp;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
		{
			USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			tmp=USART_ReceiveData(USART1);
			xQueueSendFromISR(xRxCMDQue, (void *)&tmp, &xHigherPriorityTaskWoken);
			//GPIOE->ODR ^= GPIO_Pin_7;
			//printf("rec %c\r\n", tmp);
			
		}
	if(USART_GetITStatus(USART1,USART_IT_TXE) != RESET)
		{
			USART_ClearITPendingBit(USART1, USART_IT_TXE);
			//GPIOE->ODR ^= GPIO_Pin_7;
		}

	
	if( xHigherPriorityTaskWoken != pdFALSE )
		  {
			portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
		  }
}



/**
  * @brief  This function handles can1 output request.
  * @param  None
  * @retval None
  */
void CAN1_TX_IRQHandler(void)
{
	//DUG_PRINTF("CAN1_TX_IRQHandler+++ \r\n");
	if(CAN_GetITStatus(CAN1, CAN_TSR_RQCP0) == SET)
	{
		//DUG_PRINTF("CAN1_TSR_RQCP0 \r\n");
		Can1TransmitStatus = CAN_TransmitStatus(CAN1, Can1TransmitMailbox);
		//printf("TSR = 0x%x\r\n", CAN1->TSR);
		CAN1->TSR |= CAN_TSR_RQCP0;	
	}
	else if(CAN_GetITStatus(CAN1, CAN_TSR_RQCP1) == SET)
	{
		DUG_PRINTF("CAN1_TSR_RQCP1 \r\n");
		CAN1->TSR |= CAN_TSR_RQCP1;	
	}
	else if(CAN_GetITStatus(CAN1, CAN_TSR_RQCP2) == SET)
	{
		DUG_PRINTF("CAN1_TSR_RQCP2 \r\n");
		CAN1->TSR |= CAN_TSR_RQCP2;	
	}
//	DUG_PRINTF("CAN1_TX_IRQHandler--- \r\n");

}

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg canRxMessage;
	 portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
	{
		//DUG_PRINTF("CAN1_IT_FMP0 \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &canRxMessage);
		xQueueSendFromISR(xCan1RxDataQue, ( void * ) &canRxMessage, &xHigherPriorityTaskWoken);

		//CAN_IT_FMP0_F = 1;
		//dest_add_it = dest_add;
		
		//RMAPush(&GRxMessage);
		//Can1ReceiveStatus = 1;	
		
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FF0) == SET)
	{
		DUG_PRINTF("CAN1_IT_FF0 \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FOV0) == SET)
	{
		DUG_PRINTF("CAN1_IT_FOV0 \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);	
	}

	if( xHigherPriorityTaskWoken != pdFALSE )
	  {
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	  }
	
}

void CAN1_RX1_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP1) == SET)
	{
		DUG_PRINTF("CAN1_IT_FMP1 \r\n");
		CAN_FIFORelease(CAN1, CAN_FIFO1);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FF1) == SET)
	{
		DUG_PRINTF("CAN1_IT_FF1 \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF1);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_FOV1) == SET)
	{
		DUG_PRINTF("CAN1_IT_FOV1 \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_FOV1);	
	}
}

void CAN1_SCE_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1, CAN_IT_WKU) == SET)
	{
		DUG_PRINTF("CAN1_IT_WKU \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_WKU);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_SLK) == SET)
	{
		DUG_PRINTF("CAN1_IT_SLK \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_SLK);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_EWG) == SET)
	{
		DUG_PRINTF("CAN1_IT_EWG \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_EWG);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_EPV) == SET)
	{
		DUG_PRINTF("CAN1_IT_EPV \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_EPV);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_BOF) == SET)
	{
		DUG_PRINTF("CAN1_IT_BOF \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_LEC) == SET)
	{
		DUG_PRINTF("CAN1_IT_LEC \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);	
	}
	else if(CAN_GetITStatus(CAN1, CAN_IT_ERR) == SET)
	{
		DUG_PRINTF("CAN1_IT_ERR \r\n");
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);	
	}

}


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
