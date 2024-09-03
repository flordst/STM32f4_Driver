/*
 * stm32_gpio.h
 *
 *  Created on: Jul 21, 2024
 *      Author: Flordst
 */

#ifndef INC_STM32F4_GPIO_H_
#define INC_STM32F4_GPIO_H_
#include "stm32f4xx.h"


typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

typedef struct{
	GPIO_Register_t *pGPIOx;
	GPIO_PinConfig_t GPIO_Config;
}GPIO_Handle_t;


/*DEFINE LOCATE{ */
//PinNo
#define GPIO_PIN_0                 0  /* Pin 0 selected    */
#define GPIO_PIN_1                 1 /* Pin 1 selected    */
#define GPIO_PIN_2                 2  /* Pin 2 selected    */
#define GPIO_PIN_3                 3  /* Pin 3 selected    */
#define GPIO_PIN_4                 4  /* Pin 4 selected    */
#define GPIO_PIN_5                 5  /* Pin 5 selected    */
#define GPIO_PIN_6                 6  /* Pin 6 selected    */
#define GPIO_PIN_7                 7  /* Pin 7 selected    */
#define GPIO_PIN_8                 8  /* Pin 8 selected    */
#define GPIO_PIN_9                 9  /* Pin 9 selected    */
#define GPIO_PIN_10                10  /* Pin 10 selected   */
#define GPIO_PIN_11                11  /* Pin 11 selected   */
#define GPIO_PIN_12                12  /* Pin 12 selected   */
#define GPIO_PIN_13                13  /* Pin 13 selected   */
#define GPIO_PIN_14                14  /* Pin 14 selected   */
#define GPIO_PIN_15                15  /* Pin 15 selected   */
//

//Mode
#define GPIO_IN 	0
#define GPIO_OUT 	1
#define GPIO_ALTFN 	2
#define GPIO_ANALOG	3
#define GPIO_IT_FT	4
#define GPIO_IT_RT	5
#define GPIO_IT_RFT	6
//

//Type
#define GPIO_OP_PP 0 //Push-pull
#define GPIO_OP_OD 1 //open drain
//

//SPEED
#define GPIO_SPEED_LOW 		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_FAST 	2
#define GPIO_SPEED_HIGH	 	3
//


//PinMode
#define GPIO_NO_PUPD	0
#define GPIO_PU			1
#define GPIO_PD			2

/*} */

#define GPIO_BASEADDR_CODE(x) ((x == GPIOA) ? 0: (x == GPIOB) ? 1: (x == GPIOC) ? 2:(x == GPIOD) ? 3:(x == GPIOE) ? 4:(x == GPIOF) ? 5:(x == GPIOG) ? 6:(x == GPIOH) ? 7:(x == GPIOI) ? 8:(x == GPIOJ) ? 9:(x == GPIOK) ? 10:0)
void GPIO_Clock_Control(GPIO_Register_t *pGPIOx,uint8_t EN_DIS);
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_Deinit(GPIO_Register_t *pGPIOx);

uint8_t GPIO_ReadPin(GPIO_Register_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIO_Register_t *GPIOx);
void GPIO_WritePin(GPIO_Register_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WritePort(GPIO_Register_t *pGPIOx,uint8_t Value);
void GPIO_Toggle(GPIO_Register_t *pGPIOx,uint8_t PinNumber);

void GPIO_IRQInterrupConfig(uint8_t IRQNumber,uint8_t EN_DIS);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t Priority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /*INC_STM32F4_GPIO_H_ */
