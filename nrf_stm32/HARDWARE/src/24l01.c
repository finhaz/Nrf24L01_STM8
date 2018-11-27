#include <string.h>
#include "24l01.h"
#include "delay.h"
#include "spi.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//NRF24L01驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
    
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //发送地址
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0xFF,0xFF,0xFF,0xFF,0xFF}; //发送地址

u8 rec_buf[33]; //多一位是作为结束符，nrf最多收发32字节，字符串默认以0作为结束符
u8 tra_buf[32];

void nrf_delay_us(u16 us)
{

	u16 t;
	while(us--)
	  for(t=10;t>0;t--);

}



//初始化24L01的IO口
void NRF24L01_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);	 //使能PA B端口时钟
    		 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 //PB12上拉 防止W25X的干扰 CE
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	//初始化指定IO
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	//PA8 推挽 	  CSN
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化指定IO
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;    //IRQ
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING ; //PB5 上拉输入  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	

  SPI2_Init();    		//初始化SPI	 
 

			 
	NRF24L01_CE=0; 			//使能24L01
	NRF24L01_CSN=1;			//SPI片选取消  
	 		 	 
}
//检测24L01是否存在
//返回值:0，成功;1，失败	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}	 	 
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPI2_ReadWriteByte(reg);//发送寄存器号 
  	SPI2_ReadWriteByte(value);      //写入寄存器的值
  	NRF24L01_CSN=1;                 //禁止SPI传输	   
  	return(status);       			//返回状态值
}
//读取SPI寄存器值
//reg:要读的寄存器
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPI2_ReadWriteByte(reg);   //发送寄存器号
  	reg_val=SPI2_ReadWriteByte(0XFF);//读取寄存器内容
  	NRF24L01_CSN = 1;          //禁止SPI传输		    
  	return(reg_val);           //返回状态值
}	
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=SPI2_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)pBuf[u8_ctr]=SPI2_ReadWriteByte(0XFF);//读出数据
  	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
}
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPI2_ReadWriteByte(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)SPI2_ReadWriteByte(*pBuf++); //写入数据	 
  	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}				   
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);//spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	NRF24L01_CE=0;
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 	NRF24L01_CE=1;//启动发送	   
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//启动NRF24L01接收一次数据
//rxbuf:待接收数据首地址
//返回值:0，接收完成；1，没收到数据
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 
		return 0; 
	}	   
	return 1;//没收到任何数据
}					    
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了		   
void NRF24L01_RX_Mode(void)
{
		NRF24L01_CE=0;	  
  	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);    //使能通道0的自动应答    
//  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x00);    //关闭通道0的自动应答    	
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);	     //设置RF通信频率	,2400+ RH_CH mhz	  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG, 0x0f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 

  	NRF24L01_CE = 1; //CE为高,进入接收模式 
}						 
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入TX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	 
void NRF24L01_TX_Mode(void)
{														 
		NRF24L01_CE=0;	    
  	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 
  	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //使能通道0的自动应答    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //使能通道0的接收地址  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);       //设置RF通道为0,通讯频率2400+ RH_CH mhz
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
		NRF24L01_CE=1;//CE为高,10us后启动发送
}		  

/****NRF24L01收发模式初始化*********/
void NRF24L01_RT_Mode(void)
{

	clear_status();
	NRF24L01_CE=0;
	
  NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 	
	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //使能通道0的自动应答    
  NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //使能通道0的接收地址  
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);       //设置RF通道为0,通讯频率2400+ RH_CH mhz 双方一致
	NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x0f);    //配置基本工作模式的参数:接收模式，1~16位CRC校验，IRQ显示所有中断
	
	NRF24L01_CE=1;
	

}

/******nrf24l01收发程序*************/

u8 sta; //状态寄存器存储


void nrf_RxTx(u8 mod_nrf ,u8 *buff)
{
	static u8 mod_nrf_b; //static 地址不释放
	
	/*****进入发射模式****/
	
	//******进入发射模式******
	if(mod_nrf == 't')
	{
		if(mod_nrf_b != 't')
		{	
			mod_nrf_b = 't';
			NRF24L01_CE = 0; 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP); 	//清除中断标志
			NRF24L01_Write_Reg(FLUSH_TX,NOP);			//清除TX_FIFO寄存器 
			NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG,0x7e);//IRQ引脚不显示中断 上电 发射模式  1~16CRC校验
			NRF24L01_CE = 1;
			//nrf_delay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us		   
		}
		
//******发送数据******
		NRF24L01_CE= 0;			//StandBy I模式	
		NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // 装载接收端地址
		NRF24L01_Write_Buf(WR_TX_PLOAD,buff,TX_PLOAD_WIDTH); 			 // 装载数据
		NRF24L01_CE= 1;		 //置高CE激发数据发送
		
		nrf_delay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us	
		nrf_delay_us(100); //给发送数据一点时间	 比发送速度较快 延时可以比接收少
		nrf_delay_us(10);
		sta = NRF24L01_Read_Reg(STATUS);//读取状态寄存器的值
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);//清除对应的中断
					
		if(sta&TX_OK)//发送成功再清除tx fifo寄存器  TX_DS位为1
		{	
			NRF24L01_CE= 0;			
			NRF24L01_Write_Reg(FLUSH_TX,NOP); //清除tx fifo寄存器	//********重要*********
			NRF24L01_CE= 1;
		}								
				
	} 
//******进入接收模式******
	else if(mod_nrf == 'r')//接收模式
	{
		if(mod_nrf_b != 'r')
		{
			mod_nrf_b = 'r';
			NRF24L01_CE = 0; 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP);	//清除中断标志
			NRF24L01_Write_Reg(FLUSH_RX,NOP); 			//清除RX_FIFO寄存器
			NRF24L01_Write_Reg(WRITE_REG_NRF+ CONFIG, 0x0f);//IRQ引脚显示 RX_RD中断 上电 接收模式   1~16CRC校验   
			NRF24L01_CE = 1; 
			nrf_delay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us
		}		
		nrf_delay_us(500); //不能少 值可调  给接收数据一点时间
		sta = NRF24L01_Read_Reg(STATUS);
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);    
		if(sta&RX_OK)      //RX_DR 位为 收到数据
		{
			NRF24L01_CE = 0;  
			NRF24L01_Read_Buf(RD_RX_PLOAD,buff,RX_PLOAD_WIDTH);//读取数据 存入数组
			NRF24L01_Write_Reg(FLUSH_RX,NOP);//清除rx fifo寄存器	数据不抖动 
			NRF24L01_CE = 1;
		} 	 	
	 }

	
	
	
}






void clear_status(void)
{
	
	u8 status;
  status=NRF24L01_Read_Reg(STATUS);
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS, status);
	NRF24L01_Write_Reg(FLUSH_RX, NOP);    // 清除RX FIFO寄存器 
	NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* 清除TX FIFO寄存器 */
	
}



u8 NRF_Get_State(void)
{
	  u8 status;
		u8 rf_rec_flag;	
	  status=NRF24L01_Read_Reg(STATUS);
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS, status); /*清除TX_DS或MAX_RT、RX_DR中断标志*/ 
	  if(status & RX_OK)
		{
				NRF24L01_Read_Buf(RD_RX_PLOAD,rec_buf,RX_PLOAD_WIDTH);//读取数据
			  NRF24L01_Write_Reg(FLUSH_RX, NOP);    // 清除RX FIFO寄存器 
				printf("%s",rec_buf);
				rf_rec_flag = RX_OK; 
		}
		else if(status & MAX_TX) /* 达到最大重发次数 */
		{	
			NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* 清除TX FIFO寄存器 */
			rf_rec_flag = MAX_TX;
		}
		else if(status & TX_OK)/* 发送完成 */
		{
			NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* 清除TX FIFO寄存器 */
			rf_rec_flag = TX_OK;	
		}
		else 
			rf_rec_flag = 0;   /* 没收到任何数据 */
		
		return rf_rec_flag;
}


extern u8 setflag;
extern u8 cnt;

void sendtorec(void)
{
		
		
		while((sta&TX_OK)!=TX_OK)
		{
			nrf_RxTx('t',tra_buf);
		}
    memset(tra_buf,0,32);

    cnt=0;
    setflag=0;
    nrf_RxTx('r',rec_buf);     

    
}
