/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include <stdio.h>
#include "hook.h"
#include "uart.h"
#include "count.h"
#include "nRF24L01.h"

extern u8 setflag;

void main(void)
{


  Init_CLK();
  Init_UART1();
  
  Init_NRF();
  while(NRF24L01_Check());
  NRF24L01_RT_Mode();
  
  
  Init_Timer4();
  enableInterrupts();  
  
  while (1)
  {
    if(!setflag)
    {
      if(GPIO_ReadInputPin(NRF24L01_IRQ_PORT,NRF24L01_IRQ_PIN)==0)
         NRF_Get_State();
    }
    
    else
    {
        sendtorec();
    }
  }
  
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
