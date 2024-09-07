/*
 * stm32f4_I2C.c
 *
 *  Created on: Sep 4, 2024
 *      Author: admin
 */

#include <stm32f4_I2C.h>
#include <stm32f4_RCC.h>

static void I2C_GenerateStartCondition(I2C_Register_t *pI2Cx);
static void I2C_ExcuteAddrPhaseWrite(I2C_Register_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExcuteAddrPhaseRead(I2C_Register_t *pI2Cx,uint8_t SlaveAddr);

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle);

static void I2C_GenerateStartCondition(I2C_Register_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_ExcuteAddrPhaseWrite(I2C_Register_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr &=~(0x01);
	pI2Cx->DR =SlaveAddr;
}

static void I2C_ExcuteAddrPhaseRead(I2C_Register_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr <<1 ;
	SlaveAddr |=(0x01);
	pI2Cx->DR =SlaveAddr;
}

void I2C_ManagerAck(I2C_Register_t *pI2Cx,uint8_t EN_DIS){
	if(EN_DIS ==ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else{
		pI2Cx->CR1 &=~(1 << I2C_CR1_ACK);
	}
}

static void I2C_ClearAddrFlag(I2C_Handle_t *pI2C_Handle){
	uint32_t temp_read;
	if(pI2C_Handle->pI2Cx->SR2 & (1 <<I2C_SR2_MSL)) {
		if(pI2C_Handle->TxRxState ==I2C_BUSY_RX)  /*check device is master ?*/{
			if(pI2C_Handle->RxSize == 1){
				I2C_ManagerAck(pI2C_Handle->pI2Cx,DISABLE);
				//clear ADDR flag
				temp_read=pI2C_Handle->pI2Cx->SR1;
				temp_read=pI2C_Handle->pI2Cx->SR2;
				(void)temp_read;
			}
		}
		else {
			temp_read=pI2C_Handle->pI2Cx->SR1;
			temp_read=pI2C_Handle->pI2Cx->SR2;
			(void)temp_read;
		}
	}
	else  /*device is slave*/{
		temp_read=pI2C_Handle->pI2Cx->SR1;
	 	temp_read=pI2C_Handle->pI2Cx->SR2;
	 	(void)temp_read;
	}
}


void I2C_ClockControl(I2C_Register_t *pI2Cx,uint8_t EN_DIS){
	if(EN_DIS == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
	else {
		if(pI2Cx == I2C1){
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2){
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3){
			I2C3_PCLK_DIS();
		}
	}
}

void I2C_GenerateStopCondition(I2C_Register_t *pI2Cx){
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

void I2C_PeripheralControl(I2C_Register_t *pI2Cx,uint8_t EN_DIS){
	if (EN_DIS==ENABLE){
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else {
		pI2Cx->CR1 &=~(1<<I2C_CR1_PE);
	}
}


void I2C_Init(I2C_Handle_t *pI2C_Handle){
	uint32_t reg=0;
	I2C_ClockControl(pI2C_Handle->pI2Cx, ENABLE);

	//Ack
	reg |=pI2C_Handle->I2C_Config.ACKControl <<10;
	pI2C_Handle->pI2Cx->CR1 =reg;

	//FREQ CR2
	reg=0;
	reg |=RCC_GetPCLK1()/1000000U;
	pI2C_Handle->pI2Cx->CR2 =reg;

	//OAR1
	reg=0;
	reg |=pI2C_Handle->I2C_Config.DeviceAddress <<1;
	reg |=(1<<14); /*this bit should always be kept at 1 by software.*/
	pI2C_Handle->pI2Cx->OAR1 =reg;

	//CCR
	reg=0;
	uint16_t CCR_Value=0;
	if(pI2C_Handle->I2C_Config.SCLSpeed <=SCL_SPEED_SM) /*standard mode*/{

		CCR_Value=(RCC_GetPCLK1() /(2 *pI2C_Handle->I2C_Config.SCLSpeed));
		reg |=(CCR_Value & 0xFFF);
	}
	else /*fastmode*/{
		reg |=(1<<I2C_CCR_FS);
		reg |=(pI2C_Handle->I2C_Config.FMDutyCycle<<I2C_CCR_DUTY);
		if(pI2C_Handle->I2C_Config.FMDutyCycle == FMDUTY_2){
			CCR_Value = (RCC_GetPCLK1() /(3 * pI2C_Handle->I2C_Config.SCLSpeed));
		}
		else {
			CCR_Value = (RCC_GetPCLK1() /(25 * pI2C_Handle->I2C_Config.SCLSpeed));
		}
		reg |=(CCR_Value & 0xFFF);
	}
	pI2C_Handle->pI2Cx->CCR =reg;
	//TRISE
	if(pI2C_Handle->I2C_Config.SCLSpeed <=SCL_SPEED_SM) /*standard mode*/{
		reg =(RCC_GetPCLK1() / 1000000U) +1;
	}
	else /*fastmode*/{
		reg =(RCC_GetPCLK1() * 300 / 1000000000U) +1;
	}
	pI2C_Handle->pI2Cx->TRISE = (reg & 0x3F) ;
}

uint8_t I2C_FlagStatus(I2C_Register_t *pI2Cx,uint8_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterTransmit(I2C_Handle_t *pI2C_Handle,uint8_t *TxBuff,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr){
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_SB)); /*check SB flag ,SCL =Low*/

	I2C_ExcuteAddrPhaseWrite(pI2C_Handle->pI2Cx, SlaveAddr); /*send addr data and bit read/write*/
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_ADDR)); /*check ADDR flag ,SCL =Low*/
	I2C_ClearAddrFlag(pI2C_Handle);
	while(Len >0){
		while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_TXE)); /*check TXE flag ,SCL =Low*/
		pI2C_Handle->pI2Cx->DR =*TxBuff;
		TxBuff++;
		Len--;
	}
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_TXE));
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_BTF));

	if(Sr == I2C_DISABLE_SR)
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);
}

void I2C_MasterReceive(I2C_Handle_t *pI2C_Handle,uint8_t *RxBuff,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr){
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_SB)); /*check SB flag ,SCL =Low*/
	I2C_ExcuteAddrPhaseRead(pI2C_Handle->pI2Cx, SlaveAddr); /*send addr data and bit read/write*/
	while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_ADDR)); /*check ADDR flag ,SCL =Low*/
	if(Len == 1){
		I2C_ManagerAck(pI2C_Handle->pI2Cx, DISABLE);
		I2C_ClearAddrFlag(pI2C_Handle);
		while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_RXNE));
		if(Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
		}
		*RxBuff =pI2C_Handle->pI2Cx->DR;
	}

	if (Len >1){
		I2C_ClearAddrFlag(pI2C_Handle);
		for(uint32_t i= Len; i>0; i--){
			while( ! I2C_FlagStatus(pI2C_Handle->pI2Cx, FLAG_RXNE));
			if (i==2){
				I2C_ManagerAck(pI2C_Handle->pI2Cx, DISABLE);
				if(Sr == I2C_DISABLE_SR){
					I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
				}
			}
			*RxBuff=pI2C_Handle->pI2Cx->DR;
			RxBuff++;
		}
	}
	if(pI2C_Handle->I2C_Config.ACKControl == ENABLE ){
		I2C_ManagerAck(pI2C_Handle->pI2Cx, ENABLE);
	}
}
