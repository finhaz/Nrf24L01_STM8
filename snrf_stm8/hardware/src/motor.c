/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "motor.h"
#include "hook.h"



void Init_Timer2(void)
{

  //TIM2_DeInit();
  TIM2_TimeBaseInit(TIM2_PRESCALER_16,20000);
  
  TIM2_OC1Init(TIM2_OCMODE_PWM1,TIM2_OUTPUTSTATE_ENABLE,800,TIM2_OCPOLARITY_HIGH);
  TIM2_OC2Init(TIM2_OCMODE_PWM1,TIM2_OUTPUTSTATE_ENABLE,800,TIM2_OCPOLARITY_HIGH);
  

     




  TIM2_ARRPreloadConfig(ENABLE);
  TIM2_Cmd(ENABLE);
}