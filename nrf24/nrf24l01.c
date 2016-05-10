#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "stm32f10x_spi.h"
#include "delay.h"
#include "nrf24l01.h"


static struct _nrf24_reg {
	uint8_t config;
	uint8_t adr_width;
	uint8_t tx_pl_width;    //payload
	uint8_t rx_pl_width;    //payload
	uint8_t channel;
	uint8_t tx_adr[5];        //remote
	uint8_t rx_adr[5];        //local
} nrf24_reg;


void SPI_HWInit(void);
uint8_t WriteReg(uint8_t reg, uint8_t dat);
uint8_t ReadReg(uint8_t reg);
uint8_t WriteRegMulti(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
uint8_t ReadRegMulti(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

void NRF24L01_HWInit()
{
	GPIO_InitTypeDef GPIO_NRF24;
	/*开启相应IO端口的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//CSN, CE, IRQ
	GPIO_NRF24.GPIO_Pin = NRF_CSN_PIN;
	GPIO_NRF24.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_NRF24.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF_CSN_PORT, &GPIO_NRF24);

	GPIO_NRF24.GPIO_Pin = NRF_CE_PIN;
	GPIO_NRF24.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_NRF24.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(NRF_CE_PORT, &GPIO_NRF24);

	GPIO_NRF24.GPIO_Pin = NRF_IRQ_PIN;
	GPIO_NRF24.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_NRF24.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(NRF_IRQ_PORT, &GPIO_NRF24);

	/* 拉高csn引脚，NRF进入空闲状态 */
	NRF_CSN_HIGH();

	SPI_HWInit();
}

//=========================== SPI操作函数 =========================
void SPI_HWInit()
{
	SPI_InitTypeDef SPI_NRF24;
	GPIO_InitTypeDef GPIO_NRF24;

	/*开启相应IO端口的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //FIXME:SPI port
	/*使能SPI1时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);    //FIXME:SPI port

	/*配置 SPI_NRF_SPI的 SCK,MISO,MOSI引脚，GPIOA^5,GPIOA^6,GPIOA^7 */
	GPIO_NRF24.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;     //FIXME:SPI port
	GPIO_NRF24.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_NRF24.GPIO_Mode = GPIO_Mode_AF_PP;                         //复用功能
	GPIO_Init(GPIOA, &GPIO_NRF24);                                  //FIXME:SPI port

	SPI_NRF24.SPI_Direction = SPI_Direction_2Lines_FullDuplex;      //双线全双工
	SPI_NRF24.SPI_Mode = SPI_Mode_Master;                           //主模式
	SPI_NRF24.SPI_DataSize = SPI_DataSize_8b;                       //数据大小8位
	SPI_NRF24.SPI_CPOL = SPI_CPOL_Low;                              //时钟极性，空闲时为低
	SPI_NRF24.SPI_CPHA = SPI_CPHA_1Edge;                            //第1个边沿有效，上升沿为采样时刻
	SPI_NRF24.SPI_NSS = SPI_NSS_Soft;                               //NSS信号由软件产生
	SPI_NRF24.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;      //8分频，9MHz
	SPI_NRF24.SPI_FirstBit = SPI_FirstBit_MSB;                      //高位在前
	SPI_NRF24.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_NRF24);                                     //FIXME:SPI port

	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE);//FIXME:SPI port
}

uint8_t SPI_RW(uint8_t dat)
{
	/* 当 SPI发送缓冲器非空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* 通过 SPI1发送一字节数据 */
	SPI_I2S_SendData(SPI1, dat);

	/* 当SPI接收缓冲器为空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}

void SPI_WriteMulti(uint8_t *pBuf, uint8_t bytes)
{
	for (int byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		SPI_RW(*pBuf++); //写数据到缓冲区
}

void SPI_ReadMulti(uint8_t *pBuf, uint8_t bytes)
{
	for (int byte_cnt = 0; byte_cnt < bytes; byte_cnt++)
		pBuf[byte_cnt] = SPI_RW(0xFF); //从NRF24L01读取数据
}

//=========================== NRF24 SPI操作函数 =========================
uint8_t WriteReg(uint8_t reg, uint8_t dat)
{
	uint8_t status;

	reg |= NRF24L01_WRITE_REG;
	NRF_CSN_LOW();

	status = SPI_RW(reg);
	SPI_RW(dat);

	NRF_CSN_HIGH();

	return status;
}

uint8_t ReadReg(uint8_t reg)
{
	uint8_t reg_val;

	NRF_CSN_LOW();

	SPI_RW(reg);
	reg_val = SPI_RW(NRF24L01_NOP);

	NRF_CSN_HIGH();

	return reg_val;

}

uint8_t WriteRegMulti(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status;

	reg |= NRF24L01_WRITE_REG;
	NRF_CSN_LOW();

	status = SPI_RW(reg);
	SPI_WriteMulti(pBuf, bytes);

	NRF_CSN_HIGH();

	return status;
}

uint8_t ReadRegMulti(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status;

	NRF_CSN_LOW();

	status = SPI_RW(reg);
	SPI_ReadMulti(pBuf, bytes);

	NRF_CSN_HIGH();
	return status;
}
//////////////////////////////////////////////////////////////////////////
/////////////////////////////对外接口///////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void NRF24L01_Config()
{
    NRF_CE_LOW();

	NRF24L01_SetChannel(CHANNEL);

	nrf24_reg.adr_width = ADR_WIDTH;//01 10 11 for 3 to 5
	nrf24_reg.tx_pl_width = TX_PLOAD_WIDTH;
    nrf24_reg.rx_pl_width = RX_PLOAD_WIDTH;
	WriteReg(NRF24L01_SETUP_AW, nrf24_reg.adr_width-2);       //设置收发地址长度5Bytes
	WriteReg(NRF24L01_RX_PW_P0, nrf24_reg.tx_pl_width);     // Auto-ACK pipe
	WriteReg(NRF24L01_RX_PW_P1, nrf24_reg.rx_pl_width);     // Data payload pipe

	nrf24_reg.config = EN_CRC  | CRCO_1BYTES  | PRIM_RX  ;//| PWR_UP;
	WriteReg(NRF24L01_RF_SETUP, RF_DR_2Mbps | RF_PWR_0dBm); //设置发射速率为2Mbps，发射功率为最大值0dB
	WriteReg(NRF24L01_CONFIG, nrf24_reg.config);            //使能CRC 上电 PRIM_RX = 1
	WriteReg(NRF24L01_EN_AA, ENAA_P0  | ENAA_P1);           //auto ack p0 p1
	WriteReg(NRF24L01_EN_RXADDR, ERX_P0  | ERX_P1);         //enable rx pipe p0 p1
	WriteReg(NRF24L01_SETUP_RETR, ARD_250us(4) | ARC_(8));  //max retry and retry count
	WriteReg(NRF24L01_DYNPD, 0);                            // disable dynamic payload

	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_ClearIRQ();
}

BOOL NRF24L01_Check()
{
	uint8_t bufWrite[5] = { "12345" };
	uint8_t bufRead[5];
	uint8_t i;

	WriteRegMulti(NRF24L01_TX_ADDR, bufWrite, 5);
	ReadRegMulti(NRF24L01_TX_ADDR, bufRead, 5);

	for (i = 0; i < 5; i++) {
		if (bufRead[i] != '1' + i)
			break;
	}

	if (i == 5)
		return SUCCESS;         //MCU与NRF成功连接
	else
		return ERROR;           //MCU与NRF不正常连接
}

void NRF24L01_SetChannel(uint8_t ch)
{
	WriteReg(NRF24L01_RF_CH, RF_CH_MHz(ch));// 设置信道工作为2.4?GHZ，收发必须一致
	nrf24_reg.channel = ch;
}

void NRF24L01_SetLoaclAddress(uint8_t *adr)
{
	for (int i=0; i<nrf24_reg.adr_width; i++){
		nrf24_reg.rx_adr[i] = adr[i];
	}

	WriteRegMulti(NRF24L01_RX_ADDR_P1, nrf24_reg.rx_adr, nrf24_reg.adr_width);
}

void NRF24L01_SetRemoteAddress(uint8_t *adr)
{
	for (int i=0; i<nrf24_reg.adr_width; i++){
		nrf24_reg.tx_adr[i] = adr[i];
	}

	WriteRegMulti(NRF24L01_TX_ADDR, nrf24_reg.tx_adr, nrf24_reg.adr_width);
	WriteRegMulti(NRF24L01_RX_ADDR_P0, nrf24_reg.tx_adr, nrf24_reg.adr_width);//auto ack pipe
}

void NRF24L01_ClearIRQ()
{
	uint8_t irq_flag = RX_DR | TX_DS | MAX_RT;

	WriteReg(NRF24L01_STATUS, irq_flag); //清状态寄存器
}

void NRF24L01_ReusePayload()
{
	ReadReg(NRF24L01_REUSE_TX_PL);
}

void NRF24L01_FlushTx()
{
	ReadReg(NRF24L01_FLUSH_TX);
}

void NRF24L01_FlushRx()
{
	ReadReg(NRF24L01_FLUSH_RX);
}

uint8_t NRF24L01_GetFIFOStatus()
{
	uint8_t status;

	status = ReadReg(NRF24L01_FIFO_STATUS);

	return status & (TX_FULL | TX_EMPTY | RX_EMPTY | RX_FULL);
}

uint8_t NRF24L01_GetStatus()
{
	return ReadReg(NRF24L01_STATUS);
}

void NRF24L01_PowerUpTx()
{
	NRF_CE_LOW();
	NRF24L01_ClearIRQ();
	nrf24_reg.config &= ~PRIM_RX;//Set to tx mode
	nrf24_reg.config |= PWR_UP;
	WriteReg(NRF24L01_CONFIG, nrf24_reg.config);
	//NRF_CE_HIGH();
}

void NRF24L01_PowerUpRx()
{
	NRF_CE_LOW();
	NRF24L01_FlushRx();
	NRF24L01_ClearIRQ();
	nrf24_reg.config |= PRIM_RX;//Set to rx mode
	nrf24_reg.config |= PWR_UP;
	WriteReg(NRF24L01_CONFIG, nrf24_reg.config);
	NRF_CE_HIGH();
}

void NRF24L01_PowerDown()
{
	NRF_CE_LOW();
	nrf24_reg.config &= ~PWR_UP;
	WriteReg(NRF24L01_CONFIG, nrf24_reg.config);
}
//
// void NRF24L01_RxMode()
// {
// 	NRF_CE_LOW();
//
// 	WriteReg(NRF24L01_CONFIG, EN_CRC + CRCO_1BYTES + PRIM_RX + PWR_UP);//使能CRC 2Bytes 上电 PRIM_RX = 1
//
// 	/*CE拉高，进入接收模式*/
// 	NRF_CE_HIGH();
// }
//
// void NRF24L01_TxMode()
// {
// 	NRF_CE_LOW();
//
// 	WriteReg(NRF24L01_CONFIG, EN_CRC + CRCO_1BYTES + PRIM_TX + PWR_UP);//使能CRC 2Bytes 上电 PRIM_RX = 0
//
// }

BOOL NRF24L01_PutPayLoad(uint8_t * pBuf, uint8_t len)
{
	if (NRF24L01_GetFIFOStatus() & TX_FULL)
		return ERROR; //FIFO满了
	else{
		WriteRegMulti(NRF24L01_WR_TX_PLOAD, pBuf, len);
		return SUCCESS;
	}
}

BOOL NRF24L01_GetPayLoad(uint8_t * pBuf, uint8_t len)
{
	if (NRF24L01_GetFIFOStatus() & RX_EMPTY)
		return ERROR; //FIFO空，没有接收数据
	else{
		ReadRegMulti(NRF24L01_RD_RX_PLOAD, pBuf, len);
		return SUCCESS;
	}
}


pkgStatus_t NRF24L01_TxPkg(uint8_t* pkg)
{
	uint8_t status;

	if (NRF24L01_PutPayLoad(pkg, nrf24_reg.tx_pl_width) == SUCCESS) {
		NRF_CE_HIGH();
		delay_us(10);
		while (NRF_Read_IRQ() != 0);
		NRF_CE_LOW();

		status = NRF24L01_GetStatus();
		NRF24L01_ClearIRQ();

		if (status & TX_DS){
			return TX_SUCCESS;
		}else if (status & MAX_RT){
			return TX_MAX_RT;
		}

	} else {
		return TX_FIFO_FULL;
	}

}

pkgStatus_t NRF24L01_RxPkg(uint8_t* pkg)
{
	uint8_t status;

	status = NRF24L01_GetPayLoad((uint8_t*)pkg, nrf24_reg.rx_pl_width);

	if (status == SUCCESS){
		return RX_SUCCESS;
	}else{
		return RX_FIFO_EMPTY;
	}
}
