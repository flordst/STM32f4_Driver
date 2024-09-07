/*
 * test_uart.c
 *
 *  Created on: Aug 13, 2024
 *      Author: Flordst
 */


#include <stdio.h>
#include <stm32f4_UART.h>
#include <stm32f4_GPIO.h>

char rxBuf[8];
int a=0,b=0;

USART_Handle_t USART1_handle;

void USART1_Init(void);
void GPIO_USART_Init(void);
void USART1_IRQHandle(void);
void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle,uint8_t event);

int main(){
	GPIO_USART_Init();
	USART1_Init();
	USART_IRQ_ITConfig(IRQ_USART1, ENABLE);
	USART_PeripheralControl(USART1, ENABLE);
	while(1){
		while(USART_Receive_IT(&USART1_handle, (uint8_t *)rxBuf, 7) != READY);
		USART_Transmit_IT(&USART1_handle, (uint8_t *)rxBuf, 7);
	}
}

void USART1_Init(void){
	USART1_handle.pUSARTx=USART1;
	USART1_handle.USART_Config.BaudRate=BAUD_9600;
	USART1_handle.USART_Config.WordLength=WORD_8BITS;
	USART1_handle.USART_Config.StopBit=STOPBITS_1;
	USART1_handle.USART_Config.Parity=PARITY_DISABLE;
	USART1_handle.USART_Config.Mode=USART_TXRX;
	USART1_handle.USART_Config.HwFlowCtl=FLOW_CTR_NONE;
	USART_Init(&USART1_handle);
}

void GPIO_USART_Init(void){
	GPIO_Handle_t GPIO_USART;
	GPIO_USART.pGPIOx=GPIOA;
	GPIO_USART.GPIO_Config.GPIO_PinMode=GPIO_ALTFN;
	GPIO_USART.GPIO_Config.GPIO_PinOPType=GPIO_OP_PP;
	GPIO_USART.GPIO_Config.GPIO_PinControl=GPIO_PU;
	GPIO_USART.GPIO_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_USART.GPIO_Config.GPIO_PinAltFuncMode=7;

	//USART1_TX
	GPIO_USART.GPIO_Config.GPIO_PinNumber=GPIO_PIN_9;
	GPIO_Init(&GPIO_USART);

	//USART1_RX
	GPIO_USART.GPIO_Config.GPIO_PinNumber=GPIO_PIN_10;
	GPIO_Init(&GPIO_USART);
}

void USART1_IRQHandle(void){
	USART_IRQ_Handling(&USART1_handle);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle,uint8_t event){
	if(event ==EVENT_RX_CMPLT){
		a=1;
	}
	else if (event ==EVENT_RX_CMPLT){
		b=1;
	}

}


