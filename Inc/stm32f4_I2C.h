/*
 * stm32f4_I2C.h
 *
 *  Created on: Aug 20, 2024
 *      Author: Flordst
 */

#ifndef INC_STM32F4_I2C_H_
#define INC_STM32F4_I2C_H_
#include<stm32f4xx.h>

										/***Define bit of register****/
// CR1 register
#define I2C_CR1_PE 		0
#define I2C_CR1_SMBUS 	1
#define I2C_CR1_SMBTYPE 3
#define I2C_CR1_ENARP	4
#define I2C_CR1_ENPEC	5
#define I2C_CR1_ENGC	6
#define I2C_CR1_NOSTRETCH 7
#define I2C_CR1_START	8
#define I2C_CR1_STOP	9
#define I2C_CR1_ACK		10
#define I2C_CR1_POS		11
#define I2C_CR1_PEC		12
#define I2C_CR1_ALERT 	13
#define I2C_CR1_SWRST	15

//CR2 register
#define I2C_CR2_FREQ 	0
#define I2C_CR2_ITERREN 8
#define I2C_CR2_ITEVTEN	9
#define I2C_CR2_ITBUFEN	10
#define I2C_CR2_DMAEN	11
#define I2C_CR2_LAST	12

//OAR1 register
#define I2C_OAR1_ADD0	0
#define I2C_OAR1_ADD7_1	1
#define I2C_OAR1_ADD9_8	8
#define I2C_OAR1_ADDMODE 15

//OAR2 register
#define I2C_OAR2_ENDUAL	0
#define I2C_OAR2_ADD2	1

//SR1 register
#define I2C_SR1_SB		0
#define I2C_SR1_ADDR	1
#define I2C_SR1_BTF		2
#define I2C_SR1_ADD10	3
#define I2C_SR1_STOPF	4
#define I2C_SR1_RxNE	6
#define I2C_SR1_TxNE	7
#define I2C_SR1_BERR	8
#define I2C_SR1_ARLO	9
#define I2C_SR1_AF		10
#define I2C_SR1_OVR		11
#define I2C_SR1_PECRR	12
#define I2C_SR1_TIMEOUT	14
#define I2C_SR1_SMBALERT 15

//SR2 register
#define I2C_SR2_MSL	 0
#define I2C_SR2_BUSY 1
#define I2C_SR2_TRA  2
#define I2C_SR2_GENCALL	4
#define I2C_SR2_SMBDEFAUT 5
#define I2C_SR2_SMBHOST	6
#define I2C_SR2_DUALF	7

//CCR register
#define I2C_CCR_CCR	0
#define I2C_CCR_DUTY 14
#define I2C_CCR_FS 15

								/***Config and Handle***/
typedef struct{
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t ACKControl;
	uint8_t FMDutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_Register_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t RxSize;
	uint8_t DeviceAddr;
	uint8_t Sr ; /*repeat start value*/
}I2C_Handle_t;

#define I2C_READY   0
#define I2C_BUSY_RX	1
#define I2C_BUSY_TX	2

//SCL Speed
#define SCL_SPEED_SM  	100000U
#define SCL_SPEED_FM4K	400000U

//FMDutyCycle
#define FMDUTY_2 	0
#define FMDUTY_16_9	1

//Flags
#define FLAG_TXE	  (1<<I2C_SR1_TxNE)
#define FLAG_RXNE	  (1<<I2C_SR1_RxNE)
#define FLAG_SB		  (1<<I2C_SR1_SB)
#define FLAG_OVR	  (1<<I2C_SR1_OVR)
#define FLAG_AF		  (1<<I2C_SR1_AF)
#define FLAG_ARLO	  (1<<I2C_SR1_ARLO)
#define FLAG_BERR	  (1<<I2C_SR1_BERR)
#define FLAG_STOPF	  (1<<I2C_SR1_STOPF)
#define FLAG_ADD10	  (1<<I2C_SR1_ADD10)
#define FLAG_BTF	  (1<<I2C_SR1_BTF)
#define FLAG_ADDR	  (1<<I2C_SR1_ADDR)
#define FLAG_TIMEOUT  (1<<I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR RESET
#define I2C_ENABLE_SR  SET

void I2C_ClockControl(I2C_Register_t *pI2Cx,uint8_t EN_DIS);
void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_MasterTransmit(I2C_Handle_t *pI2C_Handle,uint8_t *TxBuff,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceive(I2C_Handle_t *pI2C_Handle,uint8_t *RxBuff,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_PeripheralControl(I2C_Register_t *pI2Cx,uint8_t EN_DIS);
uint8_t I2C_FlagStatus(I2C_Register_t *pI2Cx,uint8_t FlagName);
void I2C_ManagerAck(I2C_Register_t *pI2Cx,uint8_t EN_DIS);
void I2C_GenerateStopCondition (I2C_Register_t *pI2Cx);
#endif /* INC_STM32F4_I2C_H_ */
