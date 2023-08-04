#include <string.h>
#include "nRF24L01.h"
#include "stdio.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]  = {0xff,0xff,0xff,0xff,0xff}; // Define a static TX address
const u8 RX_ADDRESS[RX_ADR_WIDTH]  = {0xff,0xff,0xff,0xff,0xff}; // Define a static RX address

//uchar  stat;		   //����ڿ�λѰַ������
//sbit	RX_DR	=sta^6;	  //��λ���
//sbit	TX_DS	=sta^5;
//sbit	MAX_RT	=sta^4;

u8 rec_buf[33]; //��һλ����Ϊ��������nrf����շ�32�ֽڣ��ַ���Ĭ����0��Ϊ������
u8 tra_buf[32];



void inerDelay_us(unsigned int n)  //��Ϊ��ʱ1US
{
    for(;n>0;n--) 
    { 
        asm("nop");  //��STM8���棬16M����_nop_() ��ʱ�� 333ns
        asm("nop");  
        asm("nop");  
        asm("nop");  
    }
}

void Init_NRF(void)
{
      //����ģ���IO�ڳ�ʼ�� 
    //MISO ��ȡ����  �������룬û���ⲿ�ж�
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



//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	
	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                // CSN low, init SPI transaction
	
        status = SPI_RW(reg);      // select register	
        SPI_RW(value);             // ..and write value to it..
	
        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                   // CSN high again
	
        return(status);            // return nRF24L01 status byte
}



//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���

u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;
	
	GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);                 // CSN low, initialize SPI communication...

	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	
        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);           // CSN high, terminate SPI communication
	
	return(reg_val);        // return register value
}



//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 

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


//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ

u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
	u8 status,byte_ctr;
	       
        GPIO_WriteLow(NRF24L01_CS_PORT,NRF24L01_CS_PIN);
	status = SPI_RW(reg);
       
      /*  if(status&0x10)//MAX_RT�ж��Ƿ���Ϊ�ӷ�����������ж�
	//PB_ODR ^= 0b00100000;			
	NRF24L01_Read_Reg(WRITE_REG+STATUS,0x70);//Ҫ�����д1��������IRQ_PINΪ�ߵ�ƽ*/	
        

	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 
		SPI_RW(*pBuf++);

        GPIO_WriteHigh(NRF24L01_CS_PORT,NRF24L01_CS_PIN);

	return(status);          
}

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������

void NRF24L01_RX_Mode(void)
{
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);        //CE = 0; 
        
        
        //NRF24L01_Write_Buf(WRITE_REG + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
        NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // Use the same address on the RX device as the TX device	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);      //Ƶ��0�Զ�	ACKӦ������	 
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  //������յ�ַֻ��Ƶ��0�� 
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 0);        //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	NRF24L01_Write_Reg(WRITE_REG_NRF + RX_PW_P0, RX_PLOAD_WIDTH); //���ý������ݳ��ȣ���������Ϊ32�ֽ�
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f); // ���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB ,����������
        NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0f);    //power up  1: PRX  ���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
        
	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);       //CE = 1; 
	//inerDelay_us(130);
        
}



//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������TXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	

void NRF24L01_TX_Mode(void)
{
	GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);	
	
        NRF24L01_Write_Buf(WRITE_REG_NRF + TX_ADDR, (u8*)TX_ADDRESS, TX_ADR_WIDTH);    // д���ص�ַ	
	NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // д���ն˵�ַ	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_AA, 0x01);      //  Ƶ��0�Զ�	ACKӦ������	
	NRF24L01_Write_Reg(WRITE_REG_NRF + EN_RXADDR, 0x01);  //  ������յ�ַֻ��Ƶ��0�������Ҫ��Ƶ�����Բο�Page21  
	NRF24L01_Write_Reg(WRITE_REG_NRF + SETUP_RETR, 0x1a); // �����Զ��ط�ʱ��ʹ�����500us + 86us, 10 retrans...
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_CH, 0);        //   �����ŵ�����Ϊ2.4GHZ���շ�����һ��
	NRF24L01_Write_Reg(WRITE_REG_NRF + RF_SETUP, 0x0f);   		//���÷�������Ϊ2MHZ�����书��Ϊ���ֵ0dB,���������濪��
	NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG, 0x0e);//0x0E);���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
        
        GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
}





/****NRF24L01�շ�ģʽ��ʼ��*********/
void NRF24L01_RT_Mode(void)
{

      clear_status();
      
      GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
	
      NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 	
      NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
      NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
      NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
      NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0);       //����RFͨ��Ϊ0,ͨѶƵ��2400+ RH_CH mhz ˫��һ��
      NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
      NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
      NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x0f);    //���û�������ģʽ�Ĳ���:����ģʽ��1~16λCRCУ�飬IRQ��ʾ�����ж�
	
      GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
	

}



//����NRF24L01����һ������
//rxbuf:���ݴ洢�׵�ַ
//����ֵ:0��������ɣ�1��δ���յ�����

u8 NRF24L01_RxPacket(u8 * rx_buf)
{
    u8  stat;
    
    stat=NRF24L01_Read_Reg(STATUS);	// read register STATUS's value
    
    if(stat&RX_OK)	// if receive data ready (RX_DR) interrupt
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD,rx_buf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer
        NRF24L01_Write_Reg(FLUSH_RX,0xff);  //��� RX FIFO�Ĵ���
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

//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������TXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.



**************************************************/
u8 NRF24L01_TxPacket(u8 * tx_buf)
{
        u8  stat;
        
        GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
 	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     
	GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
              
	while(GPIO_ReadInputPin(NRF24L01_IRQ_PORT, NRF24L01_IRQ_PIN)!=0);//�ȴ������ж�
	stat=NRF24L01_Read_Reg(STATUS);//��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,stat); //���TX_DS��MAX_RT�жϱ�־
        
	if(stat&MAX_TX)//�ﵽ����ط�����
        {
          NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
          return MAX_TX; 
        }
	if(stat&TX_OK)//�������
        {
          return TX_OK;
        }
	return 0xff;//����ԭ����ʧ��
}
	

/**************************************************/

//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ���ͬʱ����ֵ:0����ʾ��λ;���򷵻�1����ʾ����λ	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0xa9,0xa9,0xa9,0xa9,0xa9};
	u8 i;
        
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5);              //����д��ĵ�ַ  
	
	for(i=0;i<5;i++)
		if(buf[i]!=0xA9)
			break;					   
	if(i!=5)
		return 1;                               //NRF24L01����λ	
	return 0;		                                //NRF24L01��λ
}	 	


/******nrf24l01�շ�����*************/

u8 sta; //״̬�Ĵ����洢


void nrf_RxTx(u8 mod_nrf,u8 *buf)
{
	static u8 mod_nrf_b; //static ��ַ���ͷ�
	
	/*****���뷢��ģʽ****/
	
	//******���뷢��ģʽ******
	if(mod_nrf == 't')
	{
		if(mod_nrf_b != 't')
		{	
			mod_nrf_b = 't';
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP); 	//����жϱ�־
			NRF24L01_Write_Reg(FLUSH_TX,NOP);			//���TX_FIFO�Ĵ��� 
			NRF24L01_Write_Reg(WRITE_REG_NRF + CONFIG,0x7e);//IRQ���Ų���ʾ�ж� �ϵ� ����ģʽ  1~16CRCУ��
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
			//nrf_delay_us(130);//��CE = 0 �� CE = 1��������ģʽ���շ�ģʽ����Ҫ���130us		   
		}
		
//******��������******
		GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);			//StandBy Iģʽ	
		NRF24L01_Write_Buf(WRITE_REG_NRF + RX_ADDR_P0, (u8*)RX_ADDRESS, RX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
		NRF24L01_Write_Buf(WR_TX_PLOAD,buf,TX_PLOAD_WIDTH); 			 // װ������
		GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);		 //�ø�CE�������ݷ���
		
		inerDelay_us(130);//��CE = 0 �� CE = 1��������ģʽ���շ�ģʽ����Ҫ���130us	
		inerDelay_us(100); //����������һ��ʱ��	 �ȷ����ٶȽϿ� ��ʱ���ԱȽ�����
		inerDelay_us(10);
		sta = NRF24L01_Read_Reg(STATUS);//��ȡ״̬�Ĵ�����ֵ
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);//�����Ӧ���ж�
					
		if(sta&TX_OK)//���ͳɹ������tx fifo�Ĵ���  TX_DSλΪ1
		{	
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);		
			NRF24L01_Write_Reg(FLUSH_TX,NOP); //���tx fifo�Ĵ���	//********��Ҫ*********
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
		}								
				
	} 
//******�������ģʽ******
	else if(mod_nrf == 'r')//����ģʽ
	{
		if(mod_nrf_b != 'r')
		{
			mod_nrf_b = 'r';
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,NOP);	//����жϱ�־
			NRF24L01_Write_Reg(FLUSH_RX,NOP); 			//���RX_FIFO�Ĵ���
			NRF24L01_Write_Reg(WRITE_REG_NRF+ CONFIG, 0x0f);//IRQ������ʾ RX_RD�ж� �ϵ� ����ģʽ   1~16CRCУ��   
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN); 
			inerDelay_us(130);//��CE = 0 �� CE = 1��������ģʽ���շ�ģʽ����Ҫ���130us
		}		
		inerDelay_us(500); //������ ֵ�ɵ�  ����������һ��ʱ��
                
		sta = NRF24L01_Read_Reg(STATUS);
		NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta);    
               
		if(sta&RX_OK)      //RX_DR λΪ �յ�����
		{
			GPIO_WriteLow(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
			NRF24L01_Read_Buf(RD_RX_PLOAD,buf,RX_PLOAD_WIDTH);//��ȡ���� ��������
			NRF24L01_Write_Reg(FLUSH_RX,NOP);//���rx fifo�Ĵ���	���ݲ����� 
			GPIO_WriteHigh(NRF24L01_CE_PORT,NRF24L01_CE_PIN);
		} 	 

	 }

	
	
	
}






void clear_status(void)
{
	
	u8 status;
        status=NRF24L01_Read_Reg(STATUS);
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS, status);
	NRF24L01_Write_Reg(FLUSH_RX, NOP);    // ���RX FIFO�Ĵ��� 
	NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* ���TX FIFO�Ĵ��� */
	
}



u8 NRF_Get_State(void)
{
	  u8 status;
	  u8 rf_rec_flag;	
	  status=NRF24L01_Read_Reg(STATUS);
	  NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS, status); /*���TX_DS��MAX_RT��RX_DR�жϱ�־*/ 
	  if(status & RX_OK)
          {
                NRF24L01_Read_Buf(RD_RX_PLOAD,rec_buf,RX_PLOAD_WIDTH);//��ȡ����
                NRF24L01_Write_Reg(FLUSH_RX, NOP);    // ���RX FIFO�Ĵ��� 
                printf("%s",rec_buf);
                memset(rec_buf,0,32);
                rf_rec_flag = RX_OK; 
          }
          else if(status & MAX_TX) /* �ﵽ����ط����� */
          {	
                  NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* ���TX FIFO�Ĵ��� */
                  rf_rec_flag = MAX_TX;
          }
          else if(status & TX_OK)/* ������� */
          {
                  NRF24L01_Write_Reg(FLUSH_TX,NOP);      /* ���TX FIFO�Ĵ��� */
                  rf_rec_flag = TX_OK;	
          }
          else 
                  rf_rec_flag = 0;   /* û�յ��κ����� */
          
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
