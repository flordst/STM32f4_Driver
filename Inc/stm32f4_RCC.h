/*
 * stm32f4_RCC.h
 *
 *  Created on: Aug 11, 2024
 *      Author: admin
 */

#ifndef INC_STM32F4_RCC_H_
#define INC_STM32F4_RCC_H_

#include <stm32f4xx.h>

uint32_t RCC_GetPCLK1(void);   //APB1 clock
uint32_t RCC_GetPCLK2(void);   //APB2 clock

#endif /* INC_STM32F4_RCC_H_ */
