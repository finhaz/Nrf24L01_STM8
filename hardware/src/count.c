/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "count.h"


//使用TIM4的定时中断

void Init_Timer4(void)
{
	/*TIM4_UpdateDisableConfig(ENABLE);//允许更新事件
	TIM4_ARRPreloadConfig(ENABLE);//自动重装
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);//中断配置，更新中断
	TIM4_SetCounter(0xff);//计数器初值
	TIM4_SetAutoreload(0xFF);//计数器自动重装的初值
	TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_UPDATE);//预分频值
	*/
  
	TIM4_TimeBaseInit(TIM4_PRESCALER_128, 125-1);   //16m  1ms一次中断
        TIM4_SetCounter(0x00);//计数器初值
        /* Clear TIM4 update flag */
        TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	 /* Enable update interrupt */
	//TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
	//TIM4_Cmd(ENABLE);
}


