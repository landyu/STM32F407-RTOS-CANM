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
typedef struct 
{
	char			flag;			//'$'分隔符
	char			flag2;			//'$'分隔符
	char			flag3;			//'$'分隔符
	char			flag4;			//'$'分隔符
	char			type;			//数据包类型
	char			id;				//车位或LED ID		
	unsigned short	region_id;		//中继器ID
	unsigned short	param1;			//0没车,1有车,2LED 当type=PARK_PACKET_TYPE_LED_SET时param1为空车位
	unsigned short	param2;			//总数
	unsigned short  param3;			//车位信息所属的区域(跟中继器ID不一样)
	unsigned short	check;			//
	char			end;			//'@'分隔符
	char			end2;			//'@'分隔符
	char			end3;			//'@'分隔符
	char			end4;			//'@'分隔符
}park_header;
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
