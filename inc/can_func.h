/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef	 __CAN_FUNC_H__
#define  __CAN_FUNC_H__

#ifdef __cplusplus
 extern "C" {
#endif	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
//#include "stm32f4xx_can.h"

#define RMALen 31
#define RMALenMask 0x1f 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported Variable ------------------------------------------------------------*/
extern vu8 Can1TransmitMailbox;
extern vu8 Can1TransmitStatus;
extern vu8 Can1ReceiveStatus;
extern CanRxMsg RxMessageArr[RMALen];
extern CanRxMsg GRxMessage;
extern vu8	 RMARPtr;
extern vu8	 RMAWPtr;
extern vu8 CAN_IT_FMP0_F;

extern xQueueHandle xCan1RxDataQue;
extern xQueueHandle xRxCMDQue;

/* Exported functions --------------------------------------------------------*/
int CAN1_Config(void); 

void can1Send(void * pvParameters);
void can1Rec(void * pvParameters);
void doCommand(void * pvParameters);


#ifdef __cplusplus
}
#endif	  	

#endif
