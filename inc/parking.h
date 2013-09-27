/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __PARKING_H__
#define  __PARKING_H__

#ifdef __cplusplus
extern "C" {
#endif    
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
#define PARK_PACKET_TYPE_CHECK_ALL			0   //要求所有点上报,包括LED
#define PARK_PACKET_TYPE_CHECK_ALL_B		1	//所有点上报回应包,每条包含一个点信息
#define PARK_PACKET_TYPE_CHECK_ONE_CAR		2   //检查某个点有没有车
#define PARK_PACKET_TYPE_CHECK_ONE_CAR_B	3	//检查某个点有没有车的回应
#define PARK_PACKET_TYPE_CHECK_ONE_REGION	4	//查询某个区域车辆信息
#define PARK_PACKET_TYPE_CHECK_ONE_REGION_B	5   //查询某个区域车辆信息的回应

#define PARK_PACKET_TYPE_AUTO_UPLOAD		10  //车位信息变化时的主动上报包
#define PARK_PACKET_TYPE_AUTO_UPLOAD_B		11  //车位信息变化时的主动上报包的回应
#define PARK_PACKET_TYPE_LED_SET			20  //LED设置包

//typedef struct 
//{
//	char			flag;			//'$'分隔符
//	char			type;			//数据包类型
//	char			id;				//车位或LED ID		
//	char			region_id;		//中继器ID
//	unsigned short	param1;			//0没车,1有车,2LED 当type=PARK_PACKET_TYPE_LED_SET时param1为空车位
//	unsigned short	param2;			//总数
//	unsigned short  param3;			//车位信息所属的区域(跟中继器ID不一样)
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
