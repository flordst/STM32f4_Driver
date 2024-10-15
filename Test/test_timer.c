/*
 * test_timer.c
 *
 *  Created on: Sep 13, 2024
 *      Author: flordst
 */



#include "main.h"


void LED_Init(void);
void Timer_init(void);
GPIO_Handle_t LED_PIN;
TIM_Handle_t hTIM4;
void delay(){
	for (uint32_t i=0;i<200000;i++);
}
int main(){

	LED_Init();
	Timer_init();

	TIM_Start_IT(&hTIM4);
	TIM_SetPriority(IRQ_TIM4, 1);
	TIM_ITConfig(IRQ_TIM4, ENABLE);

	GPIO_WritePin(GPIOA, 6, 1);



//	while(1){
////		GPIO_WritePin(GPIOA, 6, 1);
//	}
}


void LED_Init(void){

	LED_PIN.pGPIOx=GPIOA;
	LED_PIN.GPIO_Config.GPIO_PinNumber=GPIO_PIN_6;
	LED_PIN.GPIO_Config.GPIO_PinMode=GPIO_OUT;
	LED_PIN.GPIO_Config.GPIO_PinSpeed=GPIO_SPEED_FAST;
	LED_PIN.GPIO_Config.GPIO_PinOPType=GPIO_OP_PP;
	LED_PIN.GPIO_Config.GPIO_PinControl=GPIO_NO_PUPD;
	GPIO_Init(&LED_PIN);
}

void Timer_init(void){
	hTIM4.TIMx=TIM4;
	hTIM4.TIM_Config.Prescale=8000-1;
	hTIM4.TIM_Config.Period=1000-1;
	hTIM4.TIM_Config.ClockDivision=0;
	hTIM4.TIM_Config.CounterMode=UpCounter;
	TIM_Init(&hTIM4);
}

void TIM_ApplicationEventCallback(void){
	GPIO_WritePin(GPIOA, 6, 0);

}
void TIM4_IRQHandler(void) {
	TIM_IRQHandler(&hTIM4);
}
