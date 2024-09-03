/*
 * stm32f4_RCC.c
 *
 *  Created on: Aug 10, 2024
 *      Author: admin
 */

#include <stm32f4_RCC.h>

#define _16MHZ_ 16000000
#define _8MHZ_ 8000000
uint16_t AHB_PreScale[8]={2,4,8,16,64,128,256,512};
uint16_t APB_PreScale[4]={2,4,8,16};
uint32_t RCC_GetPCLK1(void){
	uint32_t Pclk1,SystemClk=0;
	uint8_t ClkSrc,temp,AHB,APB1;
	ClkSrc =((RCC->CFGR >>2 ) & 0x03);
	if(ClkSrc == 0){
		SystemClk=_16MHZ_;
	}
	else if(ClkSrc == 1){
		SystemClk=_8MHZ_;
	}
/**AHB**/
	temp=((RCC->CFGR >>4) & 0xF);
	if (temp <8){
		AHB=1;
	}
	else{
		AHB=AHB_PreScale[temp-8];
	}
/**APB1**/
	temp=((RCC->CFGR >>4) & 0x7);
	if(temp<4){
		APB1=1;
	}
	else{
		APB1=APB_PreScale[temp-4];
	}
	Pclk1=(SystemClk/AHB)/APB1;
	return Pclk1;
}

uint32_t RCC_GetPCLK2(void){
	uint32_t Pclk2,SystemClk=0;
	uint8_t ClkSrc,temp,AHB,APB2;
	ClkSrc =((RCC->CFGR >>2 ) & 0x03);
	if(ClkSrc == 0){
		SystemClk=_16MHZ_;
	}
	else if(ClkSrc == 1){
		SystemClk=_8MHZ_;
	}
/**AHB**/
	temp=((RCC->CFGR >>4) & 0xF);
	if (temp <8){
		AHB=1;
	}
	else{
		AHB=AHB_PreScale[temp-8];
	}
/**APB1**/
	temp=((RCC->CFGR >>4) & 0x7);
	if(temp<4){
		APB2=1;
	}
	else{
		APB2=APB_PreScale[temp-4];
	}
	Pclk2=(SystemClk/AHB)/APB2;
	return Pclk2;
}
