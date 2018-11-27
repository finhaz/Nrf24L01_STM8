#include <string.h>
#include "nRF24L01.h"
#include "stdio.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]  = {0xff,0xff,0xff,0xff,0xff}; // Define a static TX address
const u8 RX_ADDRESS[RX_ADR_WIDTH]  = {0xff,0xff,0xff,0xff,0xff}; // Define a static RX address

//uchar  stat;		   //存放在可位寻址的区域
//sbit	RX_DR	=sta^6;	  //按位异或
//sbit	TX_DS	=sta^5;
//sbit	MAX_RT	=sta^4;

u8 rec_buf[33]; //多一位是作为结束符，nrf最多收发32字节，字符串默认以0作为结束符
u8 tra_buf[32];



void inerDelay_us(unsigned int n)  //改为延时1US
{
    for(;n>0;n--) 
    { 
        asm("nop");  //在STM8里面，16M晶振，_nop_() 延时了 333ns
        asm("nop");  
        asm("nop");  
        asm("nop");  
    }
}

void Init_NRF(void)
{
      //无线模块的IO口初始化 
    //MISO 读取数据  浮动输入，没有外部中断
    GPIO_Init(NRF24L01_CE_PORT, (GPIO_Pin_TypeDef)(NRF24L01_CE_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//ce 
    GPIO_Init(NRF24L01_CS_PORT, (GPIO_Pin_TypeDef)(NRF24L01_CS_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//cs
    GPIO_Init(NRF24L01_SCK_PORT, (GPIO_Pin_TypeDef)(NRF24L01_SCK_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//sck
    GPIO_Init(NRF24L01_MOSI_PORT, (GPIO_Pin_TypeDef)(NRF24L01_MOSI_PIN ), GPIO_MODE_OUT_PP_HIGH_FAST);//MOSI
    GPIO_Init(NRF24L01_MISO_PORT, (GPIO_Pin_TypeDef)(NRF24L01_MISO_PIN), GPIO_MODE_IN_FL_NO_IT);//MISO
    GPIO_Init(NRF24L01_IRQ_PORT, (GPIO_Pin_TypeDef)(NRF24L01_IRQ_PIN ), GPIO_MODE_IN_FL_NO_IT); //IRQ
    
    
    SPI_DeInit();
   /* Initialize SPI in Slave mode  */
    SPI_Init(SPI_FIRSTBIT_MSB, SPI_BAUDRATEPRESCALER_2, SPI_MODE_MASTER, SPI_CLOCKPOLARITY_LOW,
    SPI_CLOCKPHASE_1EDGE, SPI_DATADIRECTION_2LINES_FULLDUPLEX, SPI_NSS_SOFT,(uint8_t)0x07);  
    /* Enable the SPI*/
    SPI_Cmd(ENABLE);
    
    GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
    GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);
}




u8 SPI_RW(u8 byte)
{

      u8 retry;
       /*!< Wait until a data is transmitted */
      while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
      {
        retry++;
        if(retry>200)return 0;
      }

      /*!< Send the byte */
      SPI_SendData(byte);
      retry=0;
      
      /*!< Wait until a data is received */
      while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
      {
        retry++;
        if(retry>200)return 0;

      }
      /*!< Get the received data */
      byte = SPI_ReceiveData();
      return(byte);  
}



//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	
	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                // CSN low, init SPI transaction
	
        status = SPI_RW(reg);      // select register	
        SPI_RW(value);             // ..and write value to it..
	
        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                   // CSN high again
	
        return(status);            // return nRF24L01 status byte
}



//读取SPI寄存器值
//reg:要读的寄存器

u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;
	
	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                 // CSN low, initialize SPI communication...

	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	
        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);           // CSN high, terminate SPI communication
	
	return(reg_val);        // return register value
}



//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值 

u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;
	
	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);// Set CSN low, init SPI tranaction
	
        status = SPI_RW(reg);       		// Select register to write to and read status byte

	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		pBuf[byte_ctr] = SPI_RW(0);    
	
	GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);

	return(status);                    // return nRF24L01 status byte
}


//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值

u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;
	       
        GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);
	status = SPI_RW(reg);
       
      /*  if(status&0x10)//MAX_RT判断是否因为从发次数溢出而中断
	//PB_ODR ^= 0b00100000;			
	NRF24L01_Read_Reg(WRITE_REG+STATUS,0x70);//要用软件写1后才能清除IRQ_PIN为高电平*/	
        

	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 
		SPI_RW(*pBuf++);

        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);

	return(status);          
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了

void NRF24L01_RX_Mode(void)
{
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);        //CE = 0; 
        
        
        //NRF24L01_Write_Buf(WRITE_REG + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
        NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // Use the same address on the RX device as the TX device	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);      //频道0自动	ACK应答允许	 
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  //允许接收地址只有频道0， 
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 0);        //   设置信道工作为2.4GHZ，收发必须一致
	NRF24L01_Write_Reg(WRITE_REG_NRF + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为32字节
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f); // 设置发射速率为2MHZ，发射功率为最大值0dB ,低噪声增益
        NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0f);    //power up  1: PRX  配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
        
	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);       //CE = 1; 
	//inerDelay_us(130);
        
}



//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入TX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.	

void NRF24L01_TX_Mode(void)
{
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);	
	
        NRF24L01_Write_Buf(WRITE_REG_NRF + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址	
	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // 写接收端地址	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);      //  频道0自动	ACK应答允许	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  //  允许接收地址只有频道0，如果需要多频道可以参考Page21  
	NRF24L01_Write_Reg(WRITE_REG_NRF + SETUP_RETR, 0x1a); // 设置自动重发时间和次数：500us + 86us, 10 retrans...
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 0);        //   设置信道工作为2.4GHZ，收发必须一致
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);   		//设置发射速率为2MHZ，发射功率为最大值0dB,低噪声增益开启
	NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0e);//0x0E);配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
        
        GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
}





/****NRF24L01收发模式初始化*********/
void NRF24L01_RT_Mode(void)
{

      clear_status();
      
      GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
	
      NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址 	
      NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
      NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //使能通道0的自动应答    
      NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //使能通道0的接收地址  
      NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);       //设置RF通道为0,通讯频率2400+ RH_CH mhz 双方一致
      NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
      NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
      NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x0f);    //配置基本工作模式的参数:接收模式，1~16位CRC校验，IRQ显示所有中断
	
      GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
	

}



//启动NRF24L01接收一次数据
//rxbuf:数据存储首地址
//返回值:0，接收完成；1，未接收到数据

u8 NRF24L01_RxPacket(u8 * rx_buf)
{
    u8  stat;
    
    stat=NRF24L01_Read_Reg(STATUS);	// read register STATUS's value
    
    if(stat&RX_OK)	// if receive data ready (RX_DR) interrupt
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        NRF24L01_Write_Reg(FLUSH_RX,0xff);  //清空 RX FIFO寄存器
        return 0;//we have receive data
    }
    
    NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,stat);// clear RX_DR or TX_DS or MAX_RT interrupt flag
    
    return 1;
}

/**************************************************
Function: nRF24L01_TxPacket();

  Description:
  This function initializes one nRF24L01 device to
  TX mode, set TX address, set RX address for auto.ack,
  fill TX payload, select RF channel, datarate & TX pwr.
  PWR_UP is set, CRC(2 bytes) is enabled, & PRIM:TX.
  
	ToDo: One high pulse(>10us) on CE will now send this
	packet and expext an acknowledgment from the RX device.

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入TX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.



**************************************************/
u8 NRF24L01_TxPacket(u8 * tx_buf)
{
        u8  stat;
        
        GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
 	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     
	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
              
	while(GPIO_ReadInputPin(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN)!=0);//等待发送中断
	stat=NRF24L01_Read_Reg(STATUS);//读取状态寄存器的值	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,stat); //清除TX_DS或MAX_RT中断标志
        
	if(stat&MAX_TX)//达到最大重发次数
        {
          NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
          return MAX_TX; 
        }
	if(stat&TX_OK)//发送完成
        {
          return TX_OK;
        }
	return 0xff;//其他原因发送失败
}
	

/**************************************************/

//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，相同时返回值:0，表示在位;否则返回1，表示不在位	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0xa9,0xa9,0xa9,0xa9,0xa9};
	u8 i;
        
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5);              //读出写入的地址  
	
	for(i=0;i<5;i++)
		if(buf[i]!=0xA9)
			break;					   
	if(i!=5)
		return 1;                               //NRF24L01不在位	
	return 0;		                                //NRF24L01在位
}	 	


/******nrf24l01收发程序*************/

u8 sta; //状态寄存器存储


void nrf_RxTx(u8 mod_nrf,u8 *buf)
{
	static u8 mod_nrf_b; //static 地址不释放
	
	/*****进入发射模式****/
	
	//******进入发射模式******
	if(mod_nrf == 't')
	{
		if(mod_nrf_b != 't')
		{	
			mod_nrf_b = 't';
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP); 	//清除中断标志
			NRF24L01_Write_Reg(FLUSH_TX,NOP);			//清除TX_FIFO寄存器 
			NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG,0x7e);//IRQ引脚不显示中断 上电 发射模式  1~16CRC校验
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
			//nrf_delay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us		   
		}
		
//******发送数据******
		GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);			//StandBy I模式	
		NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // 装载接收端地址
		NRF24L01_Write_Buf(WR_TX_PLOAD,buf,TX_PLOAD_WIDTH); 			 // 装载数据
		GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);		 //置高CE激发数据发送
		
		inerDelay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us	
		inerDelay_us(100); //给发送数据一点时间	 比发送速度较快 延时可以比接收少
		inerDelay_us(10);
		sta = NRF24L01_Read_Reg(STATUS);//读取状态寄存器的值
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);//清除对应的中断
					
		if(sta&TX_OK)//发送成功再清除tx fifo寄存器  TX_DS位为1
		{	
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);		
			NRF24L01_Write_Reg(FLUSH_TX,NOP); //清除tx fifo寄存器	//********重要*********
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
		}								
				
	} 
//******进入接收模式******
	else if(mod_nrf == 'r')//接收模式
	{
		if(mod_nrf_b != 'r')
		{
			mod_nrf_b = 'r';
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP);	//清除中断标志
			NRF24L01_Write_Reg(FLUSH_RX,NOP); 			//清除RX_FIFO寄存器
			NRF24L01_Write_Reg(WRITE_REG_NRF+ CONFIG, 0x0f);//IRQ引脚显示 RX_RD中断 上电 接收模式   1~16CRC校验   
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			inerDelay_us(130);//从CE = 0 到 CE = 1；即待机模式到收发模式，需要最大130us
		}		
		inerDelay_us(500); //不能少 值可调  给接收数据一点时间
                
		sta = NRF24L01_Read_Reg(STATUS);
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);    
               
		if(sta&RX_OK)      //RX_DR 位为 收到数据
		{
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
			NRF24L01_Read_Buf(RD_RX_PLOAD,buf,RX_PLOAD_WIDTH);//读取数据 存入数组
			NRF24L01_Write_Reg(FLUSH_RX,NOP);//清除rx fifo寄存器	数据不抖动 
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
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
                memset(rec_buf,0,32);
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
