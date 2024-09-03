/*
 * stm32_gpio.c
 *
 *  Created on: Jul 21, 2024
 *      Author: Flordst
 */


#include <stm32f4_GPIO.h>


void GPIO_Clock_Control(GPIO_Register_t *pGPIOx,uint8_t EN_DIS){
	if (EN_DIS==ENABLE){
		if (pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}else if (pGPIOx==GPIOI){
			GPIOI_PCLK_EN();
		}else if (pGPIOx==GPIOJ){
			GPIOJ_PCLK_EN();
		}else if (pGPIOx==GPIOK){
			GPIOK_PCLK_EN();
		}
	}
	else{
		//nothing
	}

}
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle){
	GPIO_Clock_Control(pGPIO_Handle->pGPIOx, ENABLE);
	uint32_t temp=0;
	//config mode
	if (pGPIO_Handle->GPIO_Config.GPIO_PinMode<=GPIO_ANALOG){
		temp=(pGPIO_Handle->GPIO_Config.GPIO_PinMode<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &=~(0x3<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER|=temp;
	}
	else { /*interrup mode*/
		if(pGPIO_Handle->GPIO_Config.GPIO_PinMode==GPIO_IT_FT){
			EXTI->FTSR |= (1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);/*config FTSR*/
			EXTI->RTSR &=~(1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);/*clear RTSR bit*/
		}
		else if(pGPIO_Handle->GPIO_Config.GPIO_PinMode==GPIO_IT_RT){
			EXTI->RTSR |= (1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);/*config RTSR*/
			EXTI->FTSR &=~(1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);/*clear FTSR bit*/
		}
		else if (pGPIO_Handle->GPIO_Config.GPIO_PinMode==GPIO_IT_RFT){
			EXTI->FTSR |= (1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber);
		}
		uint8_t temp1=pGPIO_Handle->GPIO_Config.GPIO_PinNumber/4;
		uint8_t temp2=pGPIO_Handle->GPIO_Config.GPIO_PinNumber%4;
		uint8_t code=GPIO_BASEADDR_CODE(pGPIO_Handle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |=code<<(temp2*4);
		EXTI->IMR |= 1<<pGPIO_Handle->GPIO_Config.GPIO_PinNumber;
	}
	//config speed
	temp=(pGPIO_Handle->GPIO_Config.GPIO_PinSpeed<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &=~(0x3<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR|=temp;
	//config PUPD
	temp=(pGPIO_Handle->GPIO_Config.GPIO_PinControl<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &=~(0x3<<(2*pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR|=temp;
	//config optype
	temp=(pGPIO_Handle->GPIO_Config.GPIO_PinOPType<<(pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OTYPER &=~(0x1<<(pGPIO_Handle->GPIO_Config.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OTYPER|=temp;
	//config alt function
	if (pGPIO_Handle->GPIO_Config.GPIO_PinMode==GPIO_ALTFN){
	uint8_t temp1,temp2;
	temp1=pGPIO_Handle->GPIO_Config.GPIO_PinNumber/8;
	temp2=pGPIO_Handle->GPIO_Config.GPIO_PinNumber%8;
	pGPIO_Handle->pGPIOx->AFR[temp1] &=~(0xF<<(4*temp2));
	pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_Config.GPIO_PinAltFuncMode<<(4*temp2));
	}



}
void GPIO_Deinit(GPIO_Register_t *pGPIOx){
	if (pGPIOx==GPIOA){
		GPIOA_RESET();
	}else if (pGPIOx==GPIOB){
		GPIOB_RESET();
	}else if (pGPIOx==GPIOC){
		GPIOC_RESET();
	}else if (pGPIOx==GPIOD){
		GPIOD_RESET();
	}else if (pGPIOx==GPIOE){
		GPIOE_RESET();
	}else if (pGPIOx==GPIOF){
		GPIOF_RESET();
	}else if (pGPIOx==GPIOG){
		GPIOG_RESET();
	}else if (pGPIOx==GPIOH){
		GPIOH_RESET();
	}else if (pGPIOx==GPIOI){
		GPIOI_RESET();
	}else if (pGPIOx==GPIOJ){
		GPIOJ_RESET();
	}else if (pGPIOx==GPIOK){
		GPIOK_RESET();
	}
}


uint8_t GPIO_ReadPin(GPIO_Register_t *pGPIOx,uint8_t PinNumber){
	uint8_t Value;
	Value=(uint8_t)(pGPIOx->IDR>>PinNumber)&GETBIT;
	return Value;
}
uint16_t GPIO_ReadPort(GPIO_Register_t *pGPIOx){
	uint16_t Value;
	Value= (uint16_t)(pGPIOx->IDR);
	return Value;
}

void GPIO_WritePin(GPIO_Register_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if (Value==GPIO_PIN_SET){
		pGPIOx->ODR |=(1<<PinNumber);
	}
	else{
		pGPIOx->ODR &=~(1<<PinNumber);
	}
}
void GPIO_WritePort(GPIO_Register_t *pGPIOx,uint8_t Value){
	pGPIOx->ODR=Value;
}
void GPIO_Toggle(GPIO_Register_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

void GPIO_IRQInterrupConfig(uint8_t IRQNumber,uint8_t EN_DIS){
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t Priority){
 uint8_t IPR =IRQNumber /4;
 uint8_t IRQ =IRQNumber%4;
 *(NVIC_PR_BASEADDR+IPR) |=(Priority<<(8*IRQ + 4));
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear pending
	if (EXTI->PR &(1 <<PinNumber)){
		EXTI->PR |=(1 <<PinNumber);
	}
}
