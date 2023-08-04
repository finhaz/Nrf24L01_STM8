// nRF24L01.h
#ifndef _NRF_24L01_
#define _NRF_24L01_
/*--------------------------------------------------------*/
#include "stm8s.h"

/*--------------------ִ�ж����Ŷ����--------------------*/

//CE���ţ�NRF24L01ʹ�ܶ˿�
#define NRF24L01_CE_PORT   GPIOC   
#define NRF24L01_CE_PIN    GPIO_PIN_3  


//CSN���ţ�MUC�е�SPI��NSS�Ѿ�Ϊ����,����PC4�������NRF24L01��CSN
#define NRF24L01_CS_PORT   GPIOC
#define NRF24L01_CS_PIN   GPIO_PIN_4

//����ΪSPI����

#define NRF24L01_SCK_PORT  GPIOC
#define NRF24L01_SCK_PIN   GPIO_PIN_5 

#define NRF24L01_MOSI_PORT  GPIOC 
#define NRF24L01_MOSI_PIN   GPIO_PIN_6

#define NRF24L01_MISO_PORT  GPIOC
#define NRF24L01_MISO_PIN   GPIO_PIN_7

//IRQ���� �ж����ţ��ڷ�����ɣ����յ�Ӧ�𣬽������� ʱ�򣬶��ᷢ�����䣬
//���Խ���MCU���ⲿ�ж��У����������������ж�д����
#define NRF24L01_IRQ_PORT  GPIOA
#define NRF24L01_IRQ_PIN   GPIO_PIN_3 
/*-----------------------------------------------------------*/

#define TX_ADR_WIDTH    5   // 5 bytes TX address width   5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5   // 5 bytes RX address width   5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32 // 32 bytes TX payload         32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32  // 32 bytes TX payload        32�ֽڵ��û����ݿ��


//****************************************************************//
// SPI(nRF24L01) commands
#define READ_REG_NRF        0x00  // Define read command to register  �����üĴ���,��5λΪ�Ĵ�����ַ
#define WRITE_REG_NRF       0x20  // Define write command to register д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  // Define RX payload register address  ��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  // Define TX payload register address  дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  // Define flush TX register command    ���TX FIFO�Ĵ�������
#define FLUSH_RX        0xE2  // Define flush RX register command    ���RX FIFO�Ĵ�������
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command  ����ʹ����һ������,CEΪ��,���ݰ������Ϸ���
#define NOP             0xFF  // Define No Operation, might be used to read status register �ղ���,����������״̬�Ĵ���	


//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
                              //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address  
                              //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address        
                              //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  // 'Setup address width' register address         
                              //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address         
                              //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  // 'RF channel' register address                  
                              //RFͨ��,bit6:0,����ͨ��Ƶ��
#define RF_SETUP        0x06  // 'RF setup' register address                    
                              //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  // 'Status' register address
                              //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define OBSERVE_TX      0x08  // 'Observe TX' register address
                              ////���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  // 'Carrier Detect' register address
                              //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
                              //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
                              //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
                              //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
                              //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
                              //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
                              //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  // 'TX address' register address
                              //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
                              //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
                              //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
                              //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
                              //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
                              //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
                              //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
                              //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;

//************************������������ӵ�*****************************//
#define MAX_TX  	0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  //TX��������ж�
#define RX_OK   	0x40  //���յ������ж�

#define  t_tx           100   //����ط�����
#define  t_rx           100   //������մ���



//***************************************************************//
//                   FUNCTION's PROTOTYPES  //
/****************************************************************/
// void SPI_Init(u8 Mode);     // Init HW or SW SPI
 u8 SPI_RW(u8 byte);                                // ���ֽ�SPI��д

 u8 NRF24L01_Read_Reg(u8 reg);                   //��nrf�Ĵ����ж�ȡһ���ֽ�  
 u8 NRF24L01_Write_Reg(u8 reg ,u8 value);                 // дһ���ֽ�ȥnrf�Ĵ���
 u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 bytes);  // д����ֽڵ��Ĵ���
 u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 bytes);   // �ӼĴ�����ȡ���ֽ�
//*****************************************************************/

 
void inerDelay_us(unsigned int n);

void Init_NRF(void);
void NRF_RTinit(void);
void NRF24L01_TX_Mode(void);
void NRF24L01_RX_Mode(void);
void NRF24L01_RT_Mode(void);
u8 NRF24L01_RxPacket(u8 * rx_buf);
u8 NRF24L01_TxPacket(u8 * tx_buf);
u8 NRF24L01_Check(void);
u8 NRF_Get_State(void);
void clear_status(void);
void nrf_RxTx(u8 mod_nrf,u8 *buf);
void sendtorec(void);

//extern uchar const TX_ADDRESS[TX_ADR_WIDTH];//TX address
//extern uchar const RX_ADDRESS[RX_ADR_WIDTH];//;RX address


#endif   //_NRF_24L01_
