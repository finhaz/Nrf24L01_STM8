// nRF24L01.h
#ifndef _NRF_24L01_
#define _NRF_24L01_
/*--------------------------------------------------------*/
#include "stm8s.h"

/*--------------------执行端引脚定义表--------------------*/

//CE引脚：NRF24L01使能端口
#define NRF24L01_CE_PORT   GPIOC   
#define NRF24L01_CE_PIN    GPIO_PIN_3  


//CSN引脚：MUC中的SPI的NSS已经为他用,故用PC4软件控制NRF24L01的CSN
#define NRF24L01_CS_PORT   GPIOC
#define NRF24L01_CS_PIN   GPIO_PIN_4

//以下为SPI引脚

#define NRF24L01_SCK_PORT  GPIOC
#define NRF24L01_SCK_PIN   GPIO_PIN_5 

#define NRF24L01_MOSI_PORT  GPIOC 
#define NRF24L01_MOSI_PIN   GPIO_PIN_6

#define NRF24L01_MISO_PORT  GPIOC
#define NRF24L01_MISO_PIN   GPIO_PIN_7

//IRQ引脚 中断引脚，在发送完成，接收到应答，接收数据 时候，都会发送跳变，
//可以接在MCU的外部中断中，来出发控制器进行读写操作
#define NRF24L01_IRQ_PORT  GPIOA
#define NRF24L01_IRQ_PIN   GPIO_PIN_3 
/*-----------------------------------------------------------*/

#define TX_ADR_WIDTH    5   // 5 bytes TX address width   5字节的地址宽度
#define RX_ADR_WIDTH    5   // 5 bytes RX address width   5字节的地址宽度
#define TX_PLOAD_WIDTH  32 // 32 bytes TX payload         32字节的用户数据宽度
#define RX_PLOAD_WIDTH  32  // 32 bytes TX payload        32字节的用户数据宽度


//****************************************************************//
// SPI(nRF24L01) commands
#define READ_REG_NRF        0x00  // Define read command to register  读配置寄存器,低5位为寄存器地址
#define WRITE_REG_NRF       0x20  // Define write command to register 写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  // Define RX payload register address  读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  // Define TX payload register address  写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  // Define flush TX register command    清除TX FIFO寄存器命令
#define FLUSH_RX        0xE2  // Define flush RX register command    清除RX FIFO寄存器命令
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command  重新使用上一包数据,CE为高,数据包被不断发送
#define NOP             0xFF  // Define No Operation, might be used to read status register 空操作,可以用来读状态寄存器	


//***************************************************//
// SPI(nRF24L01) registers(addresses)
#define CONFIG          0x00  // 'Config' register address
                              //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address  
                              //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address        
                              //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  // 'Setup address width' register address         
                              //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address         
                              //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  // 'RF channel' register address                  
                              //RF通道,bit6:0,工作通道频率
#define RF_SETUP        0x06  // 'RF setup' register address                    
                              //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  // 'Status' register address
                              //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define OBSERVE_TX      0x08  // 'Observe TX' register address
                              ////发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  // 'Carrier Detect' register address
                              //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
                              //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
                              //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
                              //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
                              //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
                              //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
                              //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  // 'TX address' register address
                              //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
                              //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
                              //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
                              //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
                              //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
                              //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
                              //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
                              //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;

//************************以下是自行添加的*****************************//
#define MAX_TX  	0x10  //达到最大发送次数中断
#define TX_OK   	0x20  //TX发送完成中断
#define RX_OK   	0x40  //接收到数据中断

#define  t_tx           100   //最大重发次数
#define  t_rx           100   //最大重收次数



//***************************************************************//
//                   FUNCTION's PROTOTYPES  //
/****************************************************************/
// void SPI_Init(u8 Mode);     // Init HW or SW SPI
 u8 SPI_RW(u8 byte);                                // 单字节SPI读写

 u8 NRF24L01_Read_Reg(u8 reg);                   //从nrf寄存器中读取一个字节  
 u8 NRF24L01_Write_Reg(u8 reg ,u8 value);                 // 写一个字节去nrf寄存器
 u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 bytes);  // 写多个字节到寄存器
 u8 NRF24L01_Read_Buf(u8 reg, u8 *pBuf, u8 bytes);   // 从寄存器读取多字节
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
