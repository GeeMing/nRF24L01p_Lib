#ifndef _NRF2401_H_
#define _NRF2401_H_

#include "nrf24l01_regmap.h"
#include <stdint.h>
#include "stm32f10x.h"

#ifndef BOOL
#define BOOL uint8_t
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE !FALSE
#endif
#ifndef SUCCESS
#define SUCCESS TRUE
#endif
#ifndef ERROR
#define ERROR FALSE
#endif

typedef enum {
    TX_SUCCESS = 0,
    TX_TIMEOUT,
    TX_MAX_RT,
    TX_FIFO_FULL,
    RX_SUCCESS,
    RX_FIFO_EMPTY,
}pkgStatus_t;

//****************************************IO端口定义***************************************
#define NRF_CSN_PORT		GPIOB
#define NRF_CSN_PIN			GPIO_Pin_1
#define NRF_IRQ_PORT		GPIOB
#define NRF_IRQ_PIN			GPIO_Pin_0
#define NRF_CE_PORT			GPIOA
#define NRF_CE_PIN			GPIO_Pin_4

#define NRF_CSN_HIGH()      GPIO_SetBits(NRF_CSN_PORT, NRF_CSN_PIN)
#define NRF_CSN_LOW()       GPIO_ResetBits(NRF_CSN_PORT, NRF_CSN_PIN)		        //csn置低
#define NRF_CE_HIGH()	    GPIO_SetBits(NRF_CE_PORT, NRF_CE_PIN)
#define NRF_CE_LOW()	    GPIO_ResetBits(NRF_CE_PORT, NRF_CE_PIN)			      //CE置低
#define NRF_Read_IRQ()		GPIO_ReadInputDataBit(NRF_IRQ_PORT, NRF_IRQ_PIN)  //中断引脚


//|-----------------------------|
//|GND	VCC                     |
//|CE   CSN                     |
//|SCK	MOSI                    |
//|MISO	IRQ                     |
//|               XTAL          |
//|-----------------------------|

//sck   a5
//miso	a6
//mosi	a7


#define CHANNEL      	7	//设置通讯频道
#define ADR_WIDTH 	    5 // 5 uints TX & RX address width
#define TX_PLOAD_WIDTH 	32 // TX payload
#define RX_PLOAD_WIDTH 	32 // TX payload


void NRF24L01_HWInit(void);
BOOL NRF24L01_Check(void);		//检查硬件
void NRF24L01_Config(void);
void NRF24L01_SetChannel(uint8_t ch);
void NRF24L01_SetLoaclAddress(uint8_t *adr);
void NRF24L01_SetRemoteAddress(uint8_t *adr);
void NRF24L01_PowerUpTx(void);
void NRF24L01_PowerUpRx(void);
void NRF24L01_PowerDown(void);
BOOL NRF24L01_PutPayLoad(uint8_t * pBuf, uint8_t len);
BOOL NRF24L01_GetPayLoad(uint8_t * pBuf, uint8_t len);
uint8_t NRF24L01_GetFIFOStatus(void);
uint8_t NRF24L01_GetStatus(void);
void NRF24L01_ClearIRQ(void);
void NRF24L01_ReusePayload(void);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);
BOOL NRF24L01_CheckRPD(void);
pkgStatus_t NRF24L01_TxPkg(uint8_t* pkg);
pkgStatus_t NRF24L01_RxPkg(uint8_t* pkg);



#endif
