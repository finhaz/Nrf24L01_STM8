功能简介：  
基于stm8单片机和NRF24L01实现的无线串口程序；  
开发环境为IAR FOR STM8；  
实现了nrf24l01自动切换收发模式。  
编程思路如下：  
利用硬件spi来控制nrf24l01；  
默认情况下，nrf24l01处于接收模式，一旦接收到数据，就通过串口发送显示到上位机机；  
IAR版本8.3.2.5988  
更多介绍在https://blog.csdn.net/finhaz/article/details/104049453  
