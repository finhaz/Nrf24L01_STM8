/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "encoder.h"


//使用TIM1的正交编码器功能  CH1:PC6 CH2:PC7

void Init_Timer1(void)
{
  //TIM1_DeInit();
  TIM1_TimeBaseInit(0,TIM1_COUNTERMODE_UP ,65535,0);
  TIM1_EncoderInterfaceConfig(TIM1_ENCODERMODE_TI12, TIM1_ICPOLARITY_RISING, TIM1_ICPOLARITY_RISING);
  TIM1_ARRPreloadConfig(ENABLE);
  TIM1_Cmd(ENABLE);
}





EncoderType GetEncoder;

void Get_Encoder_ONE(void)
{
  int ecode;
  u8 cnth;
  u8 cntl;
  s32 CNT2_temp,CNT2_last;
  
  cnth=TIM1->CNTRH;
  cntl=TIM1->CNTRL;
  
  ecode=(cnth<<8)|cntl;
  
  GetEncoder.cnt2 = ecode;
  CNT2_last = GetEncoder.CNT2;
  CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;  
  GetEncoder.V2 = CNT2_temp - CNT2_last;		
  
  while ((s32)(GetEncoder.V2)>Vbreak)				 
  {							      
   GetEncoder.rcnt2--;					      
   CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;
   GetEncoder.V2 = CNT2_temp - CNT2_last;		 
  }							     
  while ((s32)(GetEncoder.V2)<-Vbreak)			   
  {							      
   GetEncoder.rcnt2++;					      
   CNT2_temp = GetEncoder.rcnt2 * prd + GetEncoder.cnt2;
   GetEncoder.V2 = CNT2_temp - CNT2_last;		 
  }
  GetEncoder.CNT2 = CNT2_temp;						 
  
  TIM1->CNTRH=0;
  TIM1->CNTRL=0;
	
}
