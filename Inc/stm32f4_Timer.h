/*
 * stm32f4_Timer.h
 *
 *  Created on: Sep 10, 2024
 *      Author: Florst
 */

#ifndef INC_STM32F4_TIMER_H_
#define INC_STM32F4_TIMER_H_
#include <stm32f4xx.h>


typedef struct{
	uint32_t Prescale;
	uint32_t CounterMode;
	uint32_t ClockDivision;
	uint32_t Period;

}TIM_Config_t;


typedef struct{
	TIM_Register_t *TIMx;
	TIM_Config_t TIM_Config;
}TIM_Handle_t;

//Define bit of CR1 register
#define TIM_CR1_CEN		0
#define TIM_CR1_UDIS	1
#define TIM_CR1_URS		2
#define TIM_CR1_OPM		3
#define TIM_CR1_DIR		4
#define TIM_CR1_CMS		5
#define TIM_CR1_ARPE	7
#define TIM_CR1_CKD		8

//Define bit of CR2 register
#define TIM_CR2_CCDS	3
#define TIM_CR2_MMS		4
#define TIM_CR2_TI1S	7

//Define Counter Mode
#define UpCounter 	0
#define DownCounter 1
#define UpDownCounter_mode1 2
#define UpDownCounter_mode2 3
#define UpDownCounter_mode3 4
#define OnePulse 	5

//Define bit SR register
#define TIM_SR_UIF 0x1

//Define  CKD
#define TIM_CKD_DIV1 (0<<TIM_CR1_CKD)
#define TIM_CKD_DIV2 (1<<TIM_CR1_CKD)
#define TIM_CKD_DIV4 (2<<TIM_CR1_CKD)


void TIM_Init(TIM_Handle_t *tim_handle);
void TIM_ENABLE(TIM_Handle_t *tim_handle);
void TIM_DISABLE(TIM_Handle_t *tim_handle);
void TIM_EnableInterrupt(TIM_Handle_t *tim_handle);
void TIM_IRQHandler(TIM_Handle_t *tim_handle);
void TIM_ITConfig(uint8_t IRQNumber, uint8_t EN_DIS);
void TIM_SetPriority(uint8_t IRQNumber,uint8_t Priority);
void TIM_Start_IT(TIM_Handle_t *tim_handle);
void TIM_Stop_IT(TIM_Handle_t *tim_handle);
void TIM_ApplicationEventCallback(void);


#endif /* INC_STM32F4_TIMER_H_ */
