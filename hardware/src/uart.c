/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "uart.h"
#include "stdio.h"
#include <stdarg.h>

void Init_UART1(void)
{
  UART1_DeInit();
  
  UART1_Init((u32)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
  UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
  
  UART1_Cmd(ENABLE);
}

/*
void Send(uint8_t dat)
{
  while(( UART1_GetFlagStatus(UART1_FLAG_TXE)==RESET));
  UART1_SendData8(dat);
}
*/

int putchar(int c)  
{  
  while ((UART1->SR&0x80)==0x00); 
  UART1_SendData8((u8)c);
  //UART1_sendchar((u8)c);
  return (c);  
}



 
