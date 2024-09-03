/*
 * stm32f4_UART.c
 *
 *  Created on: Aug 10, 2024
 *      Author: Flordst
 */


#include <stm32f4_UART.h>
#include <stm32f4_RCC.h>
#include <stddef.h>

void USART_ClockControl(USART_Register_t *pUSARTx,uint8_t EN_DIS){
	if(EN_DIS==ENABLE){
		if (pUSARTx==USART1){
			USART1_PCLK_EN();
		}
		else if(pUSARTx==USART2){
			USART2_PCLK_EN();
		}
		else if(pUSARTx==USART3){
			USART3_PCLK_EN();
		}
		else if(pUSARTx==UART4){
			UART4_PCLK_EN();
		}
		else if(pUSARTx==UART5){
			UART5_PCLK_EN();
		}
		else if(pUSARTx==USART6){
			USART6_PCLK_EN();
		}
		else if(pUSARTx==UART7){
			UART7_PCLK_EN();
		}
		else if(pUSARTx==UART8){
			UART8_PCLK_EN();
		}
	}
}
void USART_Init(USART_Handle_t *pUSART_Handle){
	uint32_t reg=0;
	USART_ClockControl(pUSART_Handle->pUSARTx,ENABLE );
	//Config CR1{
	if(pUSART_Handle->USART_Config.Mode==USART_ONLY_TX){
		reg |=(1<<USART_CR1_TE);
	}
	else if(pUSART_Handle->USART_Config.Mode==USART_ONLY_RX){
		reg |=(1<<USART_CR1_RE);
	}
	else if(pUSART_Handle->USART_Config.Mode==USART_TXRX){
		reg |=((1<<USART_CR1_RE) |(1<<USART_CR1_TE)) ;
	}
	//wordlength
	reg |=pUSART_Handle->USART_Config.WordLength << USART_CR1_M;
	//parity
	if(pUSART_Handle->USART_Config.Parity==PARITY_EVEN){
		reg |=(1<<USART_CR1_PCE);
	}
	else if (pUSART_Handle->USART_Config.Parity==PARITY_ODD){
		reg |=(1<<USART_CR1_PCE);
		reg |=(1<<USART_CR1_PS);
	}
	pUSART_Handle->pUSARTx->CR1=reg;
//}

//Config CR2{
	reg=0;
	reg |= pUSART_Handle->USART_Config.StopBit << USART_CR2_STOP;
	pUSART_Handle->pUSARTx->CR2=reg;
//}

//Config CR3{
	reg=0;
	if(pUSART_Handle->USART_Config.HwFlowCtl==FLOW_CTR_CTS){
		reg |=(1<<USART_CR3_CTSE);
	}
	else if(pUSART_Handle->USART_Config.HwFlowCtl==FLOW_CTR_RTS){
		reg |=(1<<USART_CR3_RTSE);
	}
	else if(pUSART_Handle->USART_Config.HwFlowCtl==FLOW_CTR_CTS_RTS){
		reg |=(1<<USART_CR3_CTSE);
		reg |=(1<<USART_CR3_RTSE);
	}
	pUSART_Handle->pUSARTx->CR3=reg;
//}
	USART_BaudRate(pUSART_Handle->pUSARTx, pUSART_Handle->USART_Config.BaudRate);
}

void USART_BaudRate(USART_Register_t *pUSARTx,uint32_t BaudRate){
	uint32_t PclkX;
	uint32_t UsartDiv;
	uint32_t Mantissa,Fraction;
	uint32_t reg=0;
	if (pUSARTx==USART1 || pUSARTx==USART6){
		PclkX=RCC_GetPCLK2();
	}
	else {
		PclkX=RCC_GetPCLK1();
	}
	/**Check oversampling**/
	if(pUSARTx->CR1 &(1<<USART_CR1_OVER8)) {
		UsartDiv=((100*PclkX) / (8*BaudRate));   /*Over sampling by 8 */
	}
	else{
		UsartDiv=((100*PclkX) / (16*BaudRate));   /*Over sampling by 16 */
	}
	Mantissa=UsartDiv / 100;
	reg |=Mantissa<<4;
	Fraction=UsartDiv % 100;
	if(pUSARTx->CR1 &(1<<USART_CR1_OVER8)) {
		Fraction=(((Fraction *8)+50) / 100) & 0x07;
	}
	else{
		Fraction=(((Fraction *16)+50) / 100) & 0x0F;
	}
	reg |= Fraction;
	pUSARTx->BRR =reg;
}

void USART_PeripheralControl(USART_Register_t *pUSARTx,uint8_t EN_DIS){
	if(EN_DIS==ENABLE){
		pUSARTx->CR1 |= (1<<13);
	}
	else{
		pUSARTx->CR1 &=~(1<<13);
	}
}

uint8_t USART_FlagStatus(USART_Register_t *pUSARTx,uint8_t Flag){
	uint8_t Status=RESET;
	if (pUSARTx->SR & Flag){
		Status =SET;
	}
	return Status;
}

void USART_Transmit(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len){
	uint16_t *pData;
	for(uint32_t i =0;i<Len;i++){
		while(! USART_FlagStatus(pUSART_Handle->pUSARTx, TXE));
		if(pUSART_Handle->USART_Config.WordLength == WORD_9BITS){
			pData= (uint16_t *)TxBuffer;
			pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t)0x1FF);
			//check Parity
			if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
				TxBuffer=TxBuffer+2;
			}
			else {
			TxBuffer ++;
			}
		}
		else {
			pUSART_Handle->pUSARTx->DR = (*TxBuffer & (uint8_t)0xFF);
			TxBuffer++;
		}
	}
	while(! USART_FlagStatus(pUSART_Handle->pUSARTx, TC));


}
void USART_Receive(USART_Handle_t *pUSART_Handle,uint8_t *RxBuffer,uint32_t Len){
	for(uint32_t i=0;i<Len;i++){
		while(! USART_FlagStatus(pUSART_Handle->pUSARTx, RXNE));
		//check wordlength
		if(pUSART_Handle->USART_Config.WordLength == WORD_9BITS){
			if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
				*((uint16_t *)RxBuffer) = pUSART_Handle->pUSARTx->DR & (uint16_t)0x1FF;
				RxBuffer = RxBuffer +2;
			}
			else{
				*RxBuffer =  pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF;
				RxBuffer++;
			}
	}
		else{
			if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
				*RxBuffer = (uint8_t) pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF;
			}
			else{
				*RxBuffer = (uint8_t) pUSART_Handle->pUSARTx->DR & (uint8_t)0x7F;
			}
			RxBuffer++;
		}
	}
}

void USART_ClearFlag(USART_Register_t *pUSARTx,uint8_t Flag){
	pUSARTx->SR &= ~(1<<Flag);

}

void USART_IRQ_ITConfig(uint8_t IRQNumber,uint8_t EN_DIS){
	if (EN_DIS==ENABLE){
			if (IRQNumber <= 31){
				*NVIC_ISER0 |=(1<<IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64){
				*NVIC_ISER1 |=(1<<(IRQNumber % 32));
			}
			else if (IRQNumber >= 64 && IRQNumber < 96){
				*NVIC_ISER2 |=(1<<(IRQNumber % 64));
			}
		}
		else{
			if (IRQNumber <= 31){
				*NVIC_ICER0 |=(1<<IRQNumber);
			}
			else if(IRQNumber > 31 && IRQNumber < 64){
				*NVIC_ICER1 |=(1<<(IRQNumber % 32));
			}
			else if (IRQNumber >= 64 && IRQNumber < 96){
				*NVIC_ICER2 |=(1<<(IRQNumber % 64));
			}
		}
}

void USART_IRQ_PriorityConfig(uint8_t IRQNumber,uint8_t Priority){
	uint8_t IPR =IRQNumber /4;
	uint8_t IRQ =IRQNumber%4;
	*(NVIC_PR_BASEADDR+IPR) |=(Priority<<(8*IRQ + 4));
}

uint8_t USART_Transmit_IT(USART_Handle_t *pUSART_Handle,uint8_t *TxBuffer,uint32_t Len){
	uint8_t TxState =pUSART_Handle->TxBusyState;
	if(TxState != BUSY_TX){
		pUSART_Handle->TxLen =Len;
		pUSART_Handle->pTxBuffer=TxBuffer;
		pUSART_Handle->TxBusyState=BUSY_TX;

		pUSART_Handle->pUSARTx->CR1 |= (1<<USART_CR1_TXEIE); 		/*Enable it for TXE */
		pUSART_Handle->pUSARTx->CR1 |= (1<<USART_CR1_TCIE); 		/*Enable it for TCIE */

	}
	return TxState;
}
uint8_t USART_Receive_IT(USART_Handle_t *pUSART_Handle,uint8_t *RxBuffer,uint32_t Len){
	uint8_t RxState =pUSART_Handle->RxBusyState;
	if(RxState != BUSY_RX){
		pUSART_Handle->RxLen =Len;
		pUSART_Handle->pRxBuffer=RxBuffer;
		pUSART_Handle->RxBusyState=BUSY_RX;
		(void)pUSART_Handle->pUSARTx->DR;
		pUSART_Handle->pUSARTx->CR1 |= (1<<USART_CR1_RXNEIE); 		/*Enable it for RXNEIE */
	}
	return RxState;
}

void USART_IRQ_Handling(USART_Handle_t *pUSART_Handle){
	uint32_t temp1, temp2;
	uint16_t *Data ;
	temp1=pUSART_Handle->pUSARTx->SR & ( 1<< USART_SR_TC);
	temp2=pUSART_Handle->pUSARTx->CR1 & (1<< USART_CR1_TCIE);
	if (temp1 && temp2){
		//if Txlen is zero ,close transmit and call application
		if (pUSART_Handle->TxBusyState ==BUSY_TX){
			if (!pUSART_Handle->TxLen){
				pUSART_Handle->pUSARTx->SR &= ~(1<<USART_SR_TC);
				pUSART_Handle->pUSARTx->CR1 &= ~(1<<USART_CR1_TCIE);
				pUSART_Handle->TxBusyState = READY;
				pUSART_Handle->pTxBuffer=NULL;
				pUSART_Handle->TxLen =0;
				USART_ApplicationEventCallback(pUSART_Handle,EVENT_TX_CMPLT);
			}
		}
	}
	temp1 =pUSART_Handle->pUSARTx->SR & (1<<USART_SR_TXE);
	temp2 =pUSART_Handle->pUSARTx->CR1 & (1<<USART_CR1_TXEIE);
	if (temp1 && temp2){
		if (pUSART_Handle->TxBusyState ==BUSY_TX){
			if(pUSART_Handle->TxLen >0){
				if (pUSART_Handle->USART_Config.WordLength == WORD_9BITS){
					Data = (uint16_t *)pUSART_Handle->pTxBuffer;
					pUSART_Handle->pUSARTx->DR = ( *Data & (uint16_t)0x1FF );
					if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
						pUSART_Handle->pTxBuffer +=2;
						pUSART_Handle->TxLen -=2;
					}
					else {
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->TxLen--;
					}
				}
				else{
					Data = (uint16_t *)pUSART_Handle->pTxBuffer;
					pUSART_Handle->pUSARTx->DR = (*Data & (uint8_t)0xFF );
					pUSART_Handle->pTxBuffer++;
					pUSART_Handle->TxLen--;
				}

			}
			if (pUSART_Handle->TxLen ==0){
				pUSART_Handle->pUSARTx->CR1 &=~(1<<USART_CR1_TXEIE);
			}
		}

	}
  //check RXNE Flag
	temp1=pUSART_Handle->pUSARTx->SR & ( 1<< USART_SR_RXNE);
	temp2=pUSART_Handle->pUSARTx->CR1 & (1<< USART_CR1_RXNEIE);
	if(temp1 && temp2){
		if (pUSART_Handle->TxBusyState ==BUSY_RX){
			if(pUSART_Handle->RxLen >0){
				if (pUSART_Handle->USART_Config.WordLength == WORD_9BITS){
					if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
					*((uint16_t *)pUSART_Handle->pRxBuffer) = (pUSART_Handle->pUSARTx->DR  & (uint16_t)0x1FF);
						pUSART_Handle->pTxBuffer +=2;
						pUSART_Handle->TxLen -=2;
					}
					else {
						*pUSART_Handle->pRxBuffer =( pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF );
						pUSART_Handle->pTxBuffer++;
						pUSART_Handle->TxLen--;
					}
				}
				else{
					if(pUSART_Handle->USART_Config.Parity == PARITY_DISABLE){
						*pUSART_Handle->pRxBuffer = (uint8_t)(pUSART_Handle->pUSARTx->DR & (uint8_t)0xFF );
					}
					else{
						*pUSART_Handle->pRxBuffer = (uint8_t)(pUSART_Handle->pUSARTx->DR & (uint8_t)0x7F );
					}
					pUSART_Handle->pRxBuffer++;
					pUSART_Handle->RxLen--;
				}
			}
			if(! pUSART_Handle->RxLen){
				pUSART_Handle->pUSARTx->CR1 &=~(1<<USART_CR1_RXNEIE);
				pUSART_Handle->RxBusyState =READY;
				USART_ApplicationEventCallback(pUSART_Handle,EVENT_RX_CMPLT);

			}
		}
	}
	temp1=pUSART_Handle->pUSARTx->SR & (1<<USART_SR_CTS);
	temp2=pUSART_Handle->pUSARTx->CR3 & (1<<USART_CR3_CTSE);
	if(temp1 && temp2){
		pUSART_Handle->pUSARTx->CR3 &=~(1<<USART_CR3_CTSE);
		USART_ApplicationEventCallback(pUSART_Handle,EVENT_CTS);

	}
	temp1=pUSART_Handle->pUSARTx->SR & (1<<USART_SR_IDLE);
	temp2=pUSART_Handle->pUSARTx->CR1 & (1<<USART_CR1_IDLEIE);
	if(temp1 && temp2){
		temp1=pUSART_Handle->pUSARTx->SR & (1<<USART_SR_IDLE);
		USART_ApplicationEventCallback(pUSART_Handle,EVENT_IDLE);
	}

	temp1=pUSART_Handle->pUSARTx->SR & (1<<USART_SR_ORE);
	temp2=pUSART_Handle->pUSARTx->CR1 & (1<<USART_CR1_RXNEIE);
	if(temp1 && temp2){
			USART_ApplicationEventCallback(pUSART_Handle,ERR_ORE);
		}
	temp2=pUSART_Handle->pUSARTx->CR3 & (1<<USART_CR3_EIE);
	if(temp2){
		temp1=pUSART_Handle->pUSARTx->SR;
		if(temp1 & (1<<ERR_FE)){
			USART_ApplicationEventCallback(pUSART_Handle,ERR_FE);

		}
		if(temp1 & (1<<ERR_ORE)){
			USART_ApplicationEventCallback(pUSART_Handle,ERR_ORE);

		}
		if(temp1 & (1<<ERR_NF)){
			USART_ApplicationEventCallback(pUSART_Handle,ERR_NF);

		}

		}
}

__attribute__((weak))void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle,uint8_t event){

}
