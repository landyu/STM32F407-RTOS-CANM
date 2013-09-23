#include "timer_func.h"
#include "stm32f4xx_tim.h"

u32  counter = 0;
u8	timeflag = 0;
u16 timer1scouter = 0;
u16 timer500mscouter = 0;
vu8	timer100mscouter = 0;
u8	timer50mscouter = 0;
u8	timer10mscouter = 0;
u8  timer1flag	= 0;
//u8  flag1s = 0;


void TI6_InterruptConfig(void);


int TI6_Config(void)
{
	//time6 clock 84M
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;	

	//RCC->APB1ENR |= ((1<<4));//enable timer6 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);
	TIM_PrescalerConfig(TIM6, 8399, TIM_PSCReloadMode_Update);
	TIM_SetAutoreload(TIM6, 10000);
	TIM_SetCounter(TIM6, 0);  //clear timx->cnt
	TIM_ARRPreloadConfig(TIM6, DISABLE);	// TIMx_ARR buffered  Auto-reload preload enable
	//TIM_UpdateDisableConfig(TIM6, DISABLE);	  //enable update event
	TIM_UpdateRequestConfig(TIM6, TIM_UpdateSource_Regular); //Only counter overflow/underflow generates an update interrupt or DMA request if enabled.

	TI6_InterruptConfig();

	TIM_Cmd(TIM6, ENABLE);  //enable timer6 conuter
	return 0;
}

void TI6_InterruptConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
}

