#ifndef _NRF24L01_REGMAP_H_
#define _NRF24L01_REGMAP_H_


//***************************************NRF24L01命令*******************************************************
#define NRF24L01_READ_REG 	0x00 // 读寄存器指令
#define NRF24L01_WRITE_REG 	0x20 // 写寄存器指令
#define NRF24L01_RD_RX_PLOAD 0x61 // 读取接收数据指令
#define NRF24L01_WR_TX_PLOAD 0xA0 // 写待发数据指令
#define NRF24L01_FLUSH_TX 	0xE1 // 冲洗发送 FIFO指令
#define NRF24L01_FLUSH_RX 	0xE2 // 冲洗接收 FIFO指令
#define NRF24L01_REUSE_TX_PL 0xE3 // 定义重复装载数据指令
#define NRF24L01_NOP 		0xFF // 保留
//*************************************寄存器地址****************************************************
#define NRF24L01_CONFIG 		0x00 // 配置收发状态，CRC校验模式以及收发状态响应方式
#define NRF24L01_EN_AA 		0x01 // 自动应答功能设置
#define NRF24L01_EN_RXADDR	0x02 // 可用信道设置
#define NRF24L01_SETUP_AW 	0x03 // 收发地址宽度设置
#define NRF24L01_SETUP_RETR 	0x04 // 自动重发功能设置
#define NRF24L01_RF_CH 		0x05 // 工作频率设置
#define NRF24L01_RF_SETUP 	0x06 // 发射速率、功耗功能设置
#define NRF24L01_STATUS 		0x07 // 状态寄存器
#define NRF24L01_OBSERVE_TX 	0x08 // 发送监测功能//TODO lost & retry pkg
#define NRF24L01_RPD 			0x09 // 载波检测 //recived power detector
#define NRF24L01_RX_ADDR_P0 	0x0A // 频道0接收数据地址
#define NRF24L01_RX_ADDR_P1 	0x0B // 频道1接收数据地址
#define NRF24L01_RX_ADDR_P2 	0x0C // 频道2接收数据地址
#define NRF24L01_RX_ADDR_P3 	0x0D // 频道3接收数据地址
#define NRF24L01_RX_ADDR_P4 	0x0E // 频道4接收数据地址
#define NRF24L01_RX_ADDR_P5 	0x0F // 频道5接收数据地址
#define NRF24L01_TX_ADDR 	0x10 // 发送地址寄存器
#define NRF24L01_RX_PW_P0 	0x11 // 接收频道0接收数据长度
#define NRF24L01_RX_PW_P1 	0x12 // 接收频道0接收数据长度
#define NRF24L01_RX_PW_P2 	0x13 // 接收频道0接收数据长度
#define NRF24L01_RX_PW_P3 	0x14 // 接收频道0接收数据长度
#define NRF24L01_RX_PW_P4 	0x15 // 接收频道0接收数据长度
#define NRF24L01_RX_PW_P5 	0x16 // 接收频道0接收数据长度
#define NRF24L01_FIFO_STATUS     0x17 // FIFO栈入栈出状态寄存器设置
#define NRF24L01_DYNPD      0x1C    //dynamic payload length
#define NRF24L01_FEATURE    0x1D    //TODO:

//**************************CONFIG寄存器(可以使用"+"位选)******************************************
#define MASK_RX_DR		0x40		//关闭RX_DR在IRQ上的中断触发，下同
#define MASK_TX_DS		0x20
#define MASK_MAX_RT		0x10
#define EN_CRC				0x08	//使能CRC
#define CRCO_2BYTES		0x04	//1: 2Bytes CRC
#define CRCO_1BYTES		0x00	//0：1Byte CRC；
#define PWR_UP				0x02	//系统上电
#define PRIM_RX				0x01	//TX/RX控制  1:PRX(主接收)  0:PTX(主发送)
#define PRIM_TX				0x00	//TX/RX控制  1:PRX(主接收)  0:PTX(主发送)
//*************************STATUS寄存器(using sta&XXX )*************************************//TODO: to accomplish
#define RX_DR		0x40 // 接收数据中断 当接收到有效数据后置一
#define TX_DS		0x20 //	数据发送完成中断 当数据发送完成后产生中断 如果工作在自动应答模式下 只有当接收到应答信号后此位置一
#define MAX_RT		0x10 // 达到最多次重发中断.如果 MAX_RT 中断产生,则必须清除后系统才能进行通讯	(以上中断均写入1清除)
#define RX_P_NO		0x0E // status[3:1] 接收数据通道号 000-101:数据通道号 110:未使用  111:RX FIFO 寄存器为空
#define P_NO0		0x00 // 000
#define P_NO1		0x01 // 001
#define P_NO2		0x02 // 010
#define P_NO3		0x03 // 011
#define P_NO4		0x04 // 100
#define P_NO5		0x05 // 101
//**************************EN_AA自动应答通道寄存器******************************************
#define ENAA_P0	0x01
#define ENAA_P1	0x02
#define ENAA_P2	0x04
#define ENAA_P3	0x08
#define ENAA_P4	0x10
#define ENAA_P5	0x20
//**************************RF_CH通讯频道******************************************
#define	RF_CH_MHz(m) ((unsigned char)m)//默认2Mhz,最大125MHz Freq = 2400Mhz + (m) MHz
//**************************EN_RXADDR通道接收使能******************************************
#define ERX_P0	0x01		//默认1
#define ERX_P1  0x02		//默认1
#define ERX_P2  0x04		//默认0
#define ERX_P3  0x08		//默认0
#define ERX_P4  0x10    //默认0
#define ERX_P5  0x20    //默认0
//**************************RF_SETUP射频设置******************************************
#define RF_DR					0x08
#define RF_DR_1Mbps		0x00
#define RF_DR_2Mbps		0x08
#define RF_PWR				0x06
#define RF_PWR_m18dBm	0x00
#define RF_PWR_m12dBm	0x02
#define RF_PWR_m6dBm	0x04
#define RF_PWR_0dBm		0x06
#define LNA_HCURR			0x01	//LNA放大
//**************************SETUP_AW收发地址长度******************************************
#define AW_3BYTES	0x01
#define AW_4BYTES	0x02
#define AW_5BYTES 0x03
//**************************SETUP_RETR自动重发设置******************************************
#define ARD						0xF0//自动重发等待时间
#define ARD_250us(t)	((t-0x01)<<4) //t * 250us+86us
#define ARC						0xF0//自动重发次数
#define ARC_(c)				((unsigned char)c) //默认0x11
//**************************FIFO_STATUS寄存器******************************************
#define TX_REUSE    0x40    //重用数据包标志
#define TX_FULL     0x20    //FIFO有三个32Bytes的payload
#define TX_EMPTY    0x10
#define RX_FULL     0x02
#define RX_EMPTY    0x01

#endif
