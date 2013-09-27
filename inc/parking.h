/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __PARKING_H__
#define  __PARKING_H__

#ifdef __cplusplus
extern "C" {
#endif    
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
#define PARK_PACKET_TYPE_CHECK_ALL			0   //Ҫ�����е��ϱ�,����LED
#define PARK_PACKET_TYPE_CHECK_ALL_B		1	//���е��ϱ���Ӧ��,ÿ������һ������Ϣ
#define PARK_PACKET_TYPE_CHECK_ONE_CAR		2   //���ĳ������û�г�
#define PARK_PACKET_TYPE_CHECK_ONE_CAR_B	3	//���ĳ������û�г��Ļ�Ӧ
#define PARK_PACKET_TYPE_CHECK_ONE_REGION	4	//��ѯĳ����������Ϣ
#define PARK_PACKET_TYPE_CHECK_ONE_REGION_B	5   //��ѯĳ����������Ϣ�Ļ�Ӧ

#define PARK_PACKET_TYPE_AUTO_UPLOAD		10  //��λ��Ϣ�仯ʱ�������ϱ���
#define PARK_PACKET_TYPE_AUTO_UPLOAD_B		11  //��λ��Ϣ�仯ʱ�������ϱ����Ļ�Ӧ
#define PARK_PACKET_TYPE_LED_SET			20  //LED���ð�

//typedef struct 
//{
//	char			flag;			//'$'�ָ���
//	char			type;			//���ݰ�����
//	char			id;				//��λ��LED ID		
//	char			region_id;		//�м���ID
//	unsigned short	param1;			//0û��,1�г�,2LED ��type=PARK_PACKET_TYPE_LED_SETʱparam1Ϊ�ճ�λ
//	unsigned short	param2;			//����
//	unsigned short  param3;			//��λ��Ϣ����������(���м���ID��һ��)
//	unsigned short    check;
//}park_header;
/* Exported constants --------------------------------------------------------*/
extern unsigned char CarInf[64];
extern volatile unsigned char DevExisting ;
extern unsigned char  DevInfo[64][64];


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void  carin(unsigned char lotnum);
void  carout(unsigned char lotnum);
void  nolot(unsigned char lotnum);

#ifdef __cplusplus
}
#endif           

#endif
