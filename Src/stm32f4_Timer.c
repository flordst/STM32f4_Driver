/*
 * stm32f4_Timer.c
 *
 *  Created on: Sep 10, 2024
 *      Author: Flordst
 */


#include <stm32f4_Timer.h>
#include<assert.h>

static void TIM_ClockControl(TIM_Register_t *TIMx,uint8_t EN_DIS){
	if(EN_DIS == ENABLE){
		if(TIMx ==TIM1){
			TIM1_PCLK_EN();
		}
		else if(TIMx==TIM2){
			TIM2_PCLK_EN();
		}
		else if(TIMx==TIM3){
			TIM3_PCLK_EN();
		}
		else if(TIMx==TIM4){
			TIM4_PCLK_EN();
		}
		else if(TIMx==TIM5){
			TIM5_PCLK_EN();
		}
		else if(TIMx==TIM6){
			TIM6_PCLK_EN();
		}
		else if(TIMx==TIM7){
			TIM7_PCLK_EN();
		}
		else if(TIMx==TIM8){
			TIM8_PCLK_EN();
		}
		else if(TIMx==TIM9){
			TIM9_PCLK_EN();
		}
		else if(TIMx==TIM10){
			TIM10_PCLK_EN();
		}
		else if(TIMx==TIM11){
			TIM11_PCLK_EN();
		}
		else if(TIMx==TIM12){
			TIM12_PCLK_EN();
		}
		else if(TIMx==TIM13){
			TIM13_PCLK_EN();
		}
		else if(TIMx==TIM14){
			TIM14_PCLK_EN();
		}
	}
}

static void TIM_SetCounterMode(TIM_Handle_t *tim_hanlde, uint8_t mode){
	tim_hanlde->TIMx->CR1 &= ~(0x60);
	tim_hanlde->TIMx->CR1 &= ~(0x10);
	if(mode==UpCounter){
		tim_hanlde->TIMx->CR1 |= ~(1<<TIM_CR1_DIR);
	}
	else if (mode ==DownCounter){
		tim_hanlde->TIMx->CR1 |= (1<<TIM_CR1_DIR);
	}
	else if(mode==UpDownCounter_mode1){
		tim_hanlde->TIMx->CR1 |= (1<<TIM_CR1_CMS);
	}
	else if(mode==UpDownCounter_mode2){
			tim_hanlde->TIMx->CR1 |= (2<<TIM_CR1_CMS);
	}
	else if(mode==UpDownCounter_mode3){
			tim_hanlde->TIMx->CR1 |= (3<<TIM_CR1_CMS);
	}
}

static void TIM_SetClockDivision(TIM_Handle_t *tim_hanlde, uint32_t clockDivision){
	if(clockDivision== 0){
			tim_hanlde->TIMx->CR1 |= TIM_CKD_DIV1;
		}
		else if(clockDivision == 2){
			tim_hanlde->TIMx->CR1 |= TIM_CKD_DIV2;
		}
		else if (clockDivision == 4){
				tim_hanlde->TIMx->CR1 |= TIM_CKD_DIV4;
		}
}

void TIM_ENABLE(TIM_Handle_t *tim_hanlde){
	tim_hanlde->TIMx->CR1 |= TIM_CR1_CEN;

}
void TIM_DISABLE(TIM_Handle_t *tim_hanlde){
	tim_hanlde->TIMx->CR1 &= ~TIM_CR1_CEN;

}

void TIM_Init(TIM_Handle_t *tim_hanlde){
	TIM_ClockControl(tim_hanlde->TIMx, ENABLE);
	//Prescale
	assert(tim_hanlde->TIM_Config.Prescale <= 65535);
	tim_hanlde->TIMx->PSC =tim_hanlde->TIM_Config.Prescale;

	//ClockDivision
	TIM_SetClockDivision(tim_hanlde, tim_hanlde->TIM_Config.ClockDivision);
	//Period
	if(tim_hanlde->TIM_Config.Period >65535){
		assert(tim_hanlde->TIMx == TIM2 || tim_hanlde->TIMx == TIM5 );
		tim_hanlde->TIMx->ARR = tim_hanlde->TIM_Config.Period;
		}
	else if(tim_hanlde->TIM_Config.Period <=65535){
		tim_hanlde->TIMx->ARR = tim_hanlde->TIM_Config.Period;
	}
	//Counter Mode
	TIM_SetCounterMode(tim_hanlde,tim_hanlde->TIM_Config.CounterMode);
	//enable timer
	TIM_ENABLE(tim_hanlde);

}

void TIM_ITConfig(uint8_t IRQNumber, uint8_t EN_DIS) {
    if (EN_DIS == ENABLE) {
        if (IRQNumber <= 31) {
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else {
        if (IRQNumber <= 31) {
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    }
}

void TIM_SetPriority(uint8_t IRQNumber,uint8_t Priority){
	uint8_t IPR =IRQNumber /4;
	uint8_t IRQ =IRQNumber%4;
	*(NVIC_PR_BASEADDR+IPR) |=(Priority<<(8*IRQ + 4));
}

void TIM_Start_IT(TIM_Handle_t *tim_handle){
	tim_handle->TIMx->DIER |=0x1;
}
void TIM_Stop_IT(TIM_Handle_t *tim_handle){
	tim_handle->TIMx->DIER &= ~0x1;

}

void TIM_IRQHandler(TIM_Handle_t *tim_handle){
	if (tim_handle->TIMx->SR & TIM_SR_UIF) {

		tim_handle->TIMx->SR &=~ TIM_SR_UIF;

		TIM_ApplicationEventCallback();
	}
}

__attribute__((weak))void TIM_ApplicationEventCallback(void){
	//do something
}

