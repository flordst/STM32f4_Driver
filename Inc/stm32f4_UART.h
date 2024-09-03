/*
 * stm32f4_UART.h
 *
 *  Created on: Aug 10, 2024
 *      Author: admin
 */

#ifndef INC_STM32F4_UART_H_
#define INC_STM32F4_UART_H_

#include <stm32f4xx.h>



typedef struct{
	uint32_t Mode;
	uint32_t BaudRate;
	uint32_t WordLength;
	uint32_t StopBit;
	uint32_t Parity;
	uint32_t HwFlowCtl;
}USART_Config_t;


typedef struct{
	USART_Register_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;

#define USART_ONLY_TX 0
#define USART_ONLY_RX 1
#define USART_TXRX 	  2

//SR register {
#define USART_SR_PE		0
#define USART_SR_FE		1
#define USART_SR_NF		2
#define USART_SR_ORE	3
#define USART_SR_IDLE	4
#define USART_SR_RXNE	5
#define USART_SR_TC		6
#define USART_SR_TXE	7
#define USART_SR_LBD	8
#define USART_SR_CTS	9
//}

//CR1 register {
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15
//}

//CR2{
#define USART_CR2_ADD	0
#define USART_CR2_LBDL	5
#define USART_CR2_LBDIE	6
#define USART_CR2_LBCL	8
#define USART_CR2_CPHA	9
#define USART_CR2_CPOL	10
#define USART_CR2_CLKEN	11
#define USART_CR2_STOP	12
#define USART_CR2_LINEN	14
//}

//CR3
#define USART_CR3_EIE	0
#define USART_CR3_IREN	1
#define USART_CR3_IRLP	2
#define USART_CR3_HDSEL	3
#define USART_CR3_NACK	4
#define USART_CR3_SCEN	5
#define USART_CR3_DMAR	6
#define USART_CR3_DMAT	7
#define USART_CR3_RTSE	8
#define USART_CR3_CTSE	9
#define USART_CR3_CTSIE	10
#define USART_CR3_ONEBIT	11

//BaudRate{
#define BAUD_1200		1200
#define BAUD_2400		2400
#define BAUD_9600		9600
#define BAUD_19200		19200
#define BAUD_38400		38400
#define BAUD_57600		57600
#define BAUD_115200		115200
#define BAUD_230400		230400
#define BAUD_460800		460800
#define BAUD_921600		921600
#define BAUD_2M			2000000
#define BAUD_3M			3000000
//}

//Parity {
#define PARITY_ODD		2
#define PARITY_EVEN		1
#define PARITY_DISABLE	0
//}

//Wordlength{
#define WORD_8BITS 0
#define WORD_9BITS 1
//}

//Stopbits{
#define STOPBITS_1		0
#define	STOPBITS_0_5	1
#define	STOPBITS_2		2
#define	STOPBITS_1_5	3
//}

//HWFlCtr{
#define FLOW_CTR_NONE	  0
#define	FLOW_CTR_CTS	  1
#define	FLOW_CTR_RTS	  2
#define	FLOW_CTR_CTS_RTS  3
//}

//Flag{
#define TXE		(1<<USART_SR_TXE)
#define RXNE	(1<<USART_SR_RXNE)
#define TC		(1<<USART_SR_TC)
//}

//State{
#define BUSY_RX 1
#define BUSY_TX 2
#define READY   0
//}

//Event{
#define EVENT_TX_CMPLT	0
#define	EVENT_RX_CMPLT	1
#define EVENT_IDLE		2
#define EVENT_CTS		3
#define EVENT_PE		4
#define ERR_FE			5
#define	ERR_NF			6
#define	ERR_ORE			7


void USART_ClockControl(USART_Register_t *pUSARTx,uint8_t EN_DIS);
void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_Transmit(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len);
void USART_Receive(USART_Handle_t *pUSART_Handle,uint8_t *RxBuffer,uint32_t Len);
uint8_t USART_Transmit_IT(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len);
uint8_t USART_Receive_IT(USART_Handle_t *pUSART_Handle,uint8_t *RxBuffer,uint32_t Len);

void USART_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EN_DIS);
void USART_IRQ_PriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void USART_IRQ_Handling(USART_Handle_t *pUSART_Handle);

uint8_t USART_FlagStatus(USART_Register_t *pUSARTx,uint8_t Flag);
void USART_ClearFlag(USART_Register_t *pUSARTx,uint8_t Flag);
void USART_PeripheralControl(USART_Register_t *pUSARTx,uint8_t EN_DIS);
void USART_BaudRate(USART_Register_t *pUSARTx,uint32_t BaudRate);

void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle,uint8_t event);

#endif /* INC_STM32F4_UART_H_ */
