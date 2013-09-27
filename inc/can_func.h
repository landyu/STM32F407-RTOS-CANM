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
	char			flag;			//'$'�ָ���
	char			flag2;			//'$'�ָ���
	char			flag3;			//'$'�ָ���
	char			flag4;			//'$'�ָ���
	char			type;			//���ݰ�����
	char			id;				//��λ��LED ID		
	unsigned short	region_id;		//�м���ID
	unsigned short	param1;			//0û��,1�г�,2LED ��type=PARK_PACKET_TYPE_LED_SETʱparam1Ϊ�ճ�λ
	unsigned short	param2;			//����
	unsigned short  param3;			//��λ��Ϣ����������(���м���ID��һ��)
	unsigned short	check;			//
	char			end;			//'@'�ָ���
	char			end2;			//'@'�ָ���
	char			end3;			//'@'�ָ���
	char			end4;			//'@'�ָ���
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
