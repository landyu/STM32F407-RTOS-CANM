#ifndef __TIMER_FUNC_H__
#define	__TIMER_FUNC_H__

#ifdef __cplusplus
extern "C" {
#endif	
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "comm.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define	F10ms	BIT0
#define	F50ms	BIT1
#define	F100ms	BIT2
#define	F500ms	BIT3
#define	F1s	    BIT4
#define	F1min	BIT5
#define	F1hour	BIT6
#define	F1day	BIT7

#define	F10msmask	BIT0_MASK
#define	F50msmask	BIT1_MASK
#define	F100msmask	BIT2_MASK
#define	F500msmask	BIT3_MASK
#define	F1smask		BIT4_MASK
#define	F1minmask	BIT5_MASK
#define	F1hourmask	BIT6_MASK
#define	F1daymask	BIT7_MASK

extern u32  counter;
extern u8	timeflag;
extern u16 	timer1scouter;
extern u16 	timer500mscouter;
extern vu8	timer100mscouter;
extern u8	timer50mscouter;
extern u8	timer10mscouter;
extern u8  	timer1flag;


/* Exported macro ------------------------------------------------------------*/
/* Exported Variable ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

int TI6_Config(void);

#ifdef __cplusplus
}
#endif	 

#endif
