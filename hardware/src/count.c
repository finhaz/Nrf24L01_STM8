/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "count.h"


//ʹ��TIM4�Ķ�ʱ�ж�

void Init_Timer4(void)
{
	/*TIM4_UpdateDisableConfig(ENABLE);//��������¼�
	TIM4_ARRPreloadConfig(ENABLE);//�Զ���װ
	TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);//�ж����ã������ж�
	TIM4_SetCounter(0xff);//��������ֵ
	TIM4_SetAutoreload(0xFF);//�������Զ���װ�ĳ�ֵ
	TIM4_PrescalerConfig(TIM4_PRESCALER_128, TIM4_PSCRELOADMODE_UPDATE);//Ԥ��Ƶֵ
	*/
  
	TIM4_TimeBaseInit(TIM4_PRESCALER_128, 125-1);   //16m  1msһ���ж�
        TIM4_SetCounter(0x00);//��������ֵ
        /* Clear TIM4 update flag */
        TIM4_ClearFlag(TIM4_FLAG_UPDATE);
	 /* Enable update interrupt */
	//TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
	//TIM4_Cmd(ENABLE);
}


