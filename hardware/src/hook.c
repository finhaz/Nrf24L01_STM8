/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "hook.h"

/*

  ���� ��ʱ��ledָʾ�� �������ⲿ��

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
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);  //ʹ���ڲ�RC������16M,  ����Ƶ ����Ƭ����Ƶ16M  
}


void Init_LED(void)
{
  GPIO_Init(GPIOB, GPIO_PIN_5,GPIO_MODE_OUT_PP_HIGH_FAST);
}




//��ʱ����   16m����³���


//---  ΢�뼶��ʱ--------------------------   
void delay_us(void)   
{    
    asm("nop"); //һ��asm("nop")��������ʾ�������Դ���100ns   
    asm("nop");   
    asm("nop");   
    asm("nop");    
}   
  
//---- ���뼶��ʱ����-----------------------   
void delay_ms(unsigned int time)   
{   
    unsigned int i;   
    while(time--)     
    for(i=900;i>0;i--)   
      delay_us();    
}  











