#include "sys.h"
#include <stdio.h>
#include <string.h> 
#include "delay.h"
#include "usart.h"
#include "24l01.h"
#include "count.h"




extern u8 setflag;



int main(void)
 {		

	delay_init();	    	 //延时函数初始化	  
	NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
	NRF24L01_Init();	 
	 
	while(NRF24L01_Check())	//检查NRF24L01是否在位	
	{
	} 
	NRF24L01_RT_Mode();
	TIM1_Int_Init(1000-1,72-1);//1ms

	
	while(1)
	{
		if(!setflag)
    {
		if(NRF24L01_IRQ==0)
			NRF_Get_State();
		}
		else
		{
			sendtorec();
		}

	 
	}
 }

 
 
