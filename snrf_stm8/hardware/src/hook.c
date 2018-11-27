/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "hook.h"

/*

  负责 延时，led指示等 板载特殊部分

*/



/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/







/* Private defines -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


void Init_CLK(void)
{
  CLK_HSECmd(DISABLE);
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);  //使用内部RC震荡器，16M,  不分频 ，单片机主频16M  
}


void Init_LED(void)
{
  GPIO_Init(GPIOB, GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
}




//延时函数   16m情况下成立


//---  微秒级延时--------------------------   
void delay_us(void)   
{    
    asm("nop"); //一个asm("nop")函数经过示波器测试代表100ns   
    asm("nop");   
    asm("nop");   
    asm("nop");    
}   
  
//---- 毫秒级延时程序-----------------------   
void delay_ms(unsigned int time)   
{   
    unsigned int i;   
    while(time--)     
    for(i=900;i>0;i--)   
      delay_us();    
}  











