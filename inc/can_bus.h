/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __CAN_BUS_H__
#define  __CAN_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif    
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
#define CanRepeater 0x00
#define SourAdd CanRepeater
#define DestAdd CanRepeater

extern unsigned char  dest_add;
extern unsigned char  dest_add_it;

extern unsigned char  SendCan;





/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


#ifdef __cplusplus
}
#endif           

#endif
