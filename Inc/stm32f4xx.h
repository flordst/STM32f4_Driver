/*
 * stm32f4xx.h
 *
 *  Created on: Jul 20, 2024
 *      Author: Flordst
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_
#include <stdio.h>

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define ROM_BASEADDR	0x1FFF0000U
#define SRAM SRAM1_BASEADDR

/*//define Bus*/
#define PERIPHERAL_BASEADDR 0x40000000U
#define APB1_BASEADDR PERIPHERAL_BASEADDR
#define APB2_BASEADDR 0x40010000U
#define AHB1_BASEADDR 0x40020000U
#define AHB2_BASEADDR 0x50000000U

/*//define GPIO (AHB1)*/
#define GPIOA_BASEADDR  (AHB1_BASEADDR+0x000)
#define	GPIOB_BASEADDR	(AHB1_BASEADDR+0x0400)
#define	GPIOC_BASEADDR	(AHB1_BASEADDR+0x0800)
#define	GPIOD_BASEADDR	(AHB1_BASEADDR+0x0C00)
#define	GPIOE_BASEADDR	(AHB1_BASEADDR+0x1000)
#define	GPIOF_BASEADDR	(AHB1_BASEADDR+0x1400)
#define	GPIOG_BASEADDR	(AHB1_BASEADDR+0x1800)
#define	GPIOH_BASEADDR	(AHB1_BASEADDR+0x1C00)
#define	GPIOI_BASEADDR	(AHB1_BASEADDR+0x2000)
#define	GPIOJ_BASEADDR	(AHB1_BASEADDR+0x2400)
#define	GPIOK_BASEADDR	(AHB1_BASEADDR+0x2800)
#define RCC_BASEADDR	(AHB1_BASEADDR+0x3800)


/*define NVIC{*/
#define NVIC_ISER0 ((volatile uint32_t *)0xE000E100)
#define NVIC_ISER1 ((volatile uint32_t *)0xE000E104)
#define NVIC_ISER2 ((volatile uint32_t *)0xE000E108)
#define NVIC_ISER3 ((volatile uint32_t *)0xE000E10C)
#define NVIC_ISER4 ((volatile uint32_t *)0xE000E110)
#define NVIC_ISER5 ((volatile uint32_t *)0xE000E114)
#define NVIC_ISER6 ((volatile uint32_t *)0xE000E118)
#define NVIC_ISER7 ((volatile uint32_t *)0xE000E11C)

#define NVIC_ICER0 ((volatile uint32_t *)0xE000E180)
#define NVIC_ICER1 ((volatile uint32_t *)0xE000E184)
#define NVIC_ICER2 ((volatile uint32_t *)0xE000E188)
#define NVIC_ICER3 ((volatile uint32_t *)0xE000E18C)
#define NVIC_ICER4 ((volatile uint32_t *)0xE000E190)
#define NVIC_ICER5 ((volatile uint32_t *)0xE000E194)
#define NVIC_ICER6 ((volatile uint32_t *)0xE000E198)
#define NVIC_ICER7 ((volatile uint32_t *)0xE000E19C)

#define NVIC_PR_BASEADDR ((volatile uint32_t *)0xE000E400)

/*}*/
									/*//define peripheral address*/
											/* ***APB1****/

//TIMER
#define TIM2_BASEADDR (APB1_BASEADDR+0x0000)
#define TIM3_BASEADDR (APB1_BASEADDR+0x0400)
#define TIM4_BASEADDR (APB1_BASEADDR+0x0800)
#define TIM5_BASEADDR (APB1_BASEADDR+0x0C00)
#define TIM6_BASEADDR (APB1_BASEADDR+0x1000)
#define TIM7_BASEADDR (APB1_BASEADDR+0x1400)
#define TIM12_BASEADDR (APB1_BASEADDR+0x1800)
#define TIM13_BASEADDR (APB1_BASEADDR+0x1C00)
#define TIM14_BASEADDR (APB1_BASEADDR+0x2000)

//UART
#define USART2_BASEADDR (APB1_BASEADDR+0x4400)
#define USART3_BASEADDR (APB1_BASEADDR+0x4800)
#define UART4_BASEADDR	(APB1_BASEADDR+0x4C00)
#define UART5_BASEADDR	(APB1_BASEADDR+0x5000)
#define UART7_BASEADDR	(APB1_BASEADDR+0x7800)
#define UART8_BASEADDR	(APB1_BASEADDR+0x7C00)

//I2C
#define I2C1_BASEADDR	(APB1_BASEADDR+0x5400)
#define I2C2_BASEADDR	(APB1_BASEADDR+0x5800)
#define I2C3_BASEADDR	(APB1_BASEADDR+0x5C00)

//CAN
#define CAN1_BASEADDR	(APB1_BASEADDR+0x6400)
#define CAN2_BASEADDR	(APB1_BASEADDR+0x6800)


									/* APB2*/

#define TIM1_BASEADDR (APB2_BASEADDR+0x0000)
#define TIM8_BASEADDR (APB2_BASEADDR+0x0400)
#define TIM9_BASEADDR (APB2_BASEADDR+0x4000)
#define TIM10_BASEADDR (APB2_BASEADDR+0x4400)
#define TIM11_BASEADDR (APB2_BASEADDR+0x4800)
#define USART1_BASEADDR (APB2_BASEADDR+0x1000)
#define USART6_BASEADDR (APB2_BASEADDR+0x1400)
#define EXTI_BASEADDR	(APB2_BASEADDR+0x3C00)
#define SYSCFG_BASEADDR	(APB2_BASEADDR+0x3800)

typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
}GPIO_Register_t;

#define GPIOA (( volatile GPIO_Register_t *)GPIOA_BASEADDR)
#define GPIOB (( volatile GPIO_Register_t *)GPIOB_BASEADDR)
#define GPIOC (( volatile GPIO_Register_t *)GPIOC_BASEADDR)
#define GPIOD (( volatile GPIO_Register_t *)GPIOD_BASEADDR)
#define GPIOE (( volatile GPIO_Register_t *)GPIOE_BASEADDR)
#define GPIOF (( volatile GPIO_Register_t *)GPIOF_BASEADDR)
#define GPIOG (( volatile GPIO_Register_t *)GPIOG_BASEADDR)
#define GPIOH (( volatile GPIO_Register_t *)GPIOH_BASEADDR)
#define GPIOI (( volatile GPIO_Register_t *)GPIOI_BASEADDR)
#define GPIOJ (( volatile GPIO_Register_t *)GPIOJ_BASEADDR)
#define GPIOK (( volatile GPIO_Register_t *)GPIOK_BASEADDR)

/*define RCC*/
typedef struct{
	volatile uint32_t CR;                              /*offset 0x00*/
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;									/*offset 0x1C*/
	volatile uint32_t APB1RSTR;                         /*offset 0x20*/
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED1[2];								/*offset 0x28-0x2C*/
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED2;	 								/*offset 0x3C*/
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED4;	 								/*offset 0x5C*/
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED5[2];	 								/*offset 0x68-0x6C*/
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED6[2];	 								/*offset 0x78-0x7C*/
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;

}RCC_Register_t;
#define RCC ((RCC_Register_t *)RCC_BASEADDR)

// EXTI
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_Register_t;
#define EXTI ((EXTI_Register_t *)EXTI_BASEADDR)

typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t  RESERVED[2];				/*offset 0x18-0x1C*/
	volatile uint32_t CMPCR;
}SYSCFG_Register_t;
#define SYSCFG ((SYSCFG_Register_t *)SYSCFG_BASEADDR)
//**UART register{
typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_Register_t;

#define USART1 ((volatile USART_Register_t *)USART1_BASEADDR)
#define USART2 ((volatile USART_Register_t *)USART2_BASEADDR)
#define USART3 ((volatile USART_Register_t *)USART3_BASEADDR)
#define UART4 ((volatile USART_Register_t *)UART4_BASEADDR)
#define UART5 ((volatile USART_Register_t *)UART5_BASEADDR)
#define USART6 ((volatile USART_Register_t *)USART6_BASEADDR)
#define UART7 ((volatile USART_Register_t *)UART7_BASEADDR)
#define UART8 ((volatile USART_Register_t *)UART8_BASEADDR)
//}

//**I2C**{
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_Register_t;

#define I2C1 (( volatile I2C_Register_t *)I2C1_BASEADDR)
#define I2C2 (( volatile I2C_Register_t *)I2C2_BASEADDR)
#define I2C3 (( volatile I2C_Register_t *)I2C3_BASEADDR)

											/*******CAN********/
/** CAN TxMailBox **/
typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;

} CAN_TxMailBox_t;

/** CAN FIFOMailBox **/
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_t;

/** CAN FilterRegister **/
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_t;

/** CAN register  **/
typedef struct
{
	volatile uint32_t MCR;
	volatile uint32_t MSR;
	volatile uint32_t TSR;
	volatile uint32_t RF0R;
	volatile uint32_t RF1R;
	volatile uint32_t IER;
	volatile uint32_t ESR;
	volatile uint32_t BTR;
	uint32_t RESERVED0[88];
	CAN_TxMailBox_t TxMailBox[3];
    CAN_FIFOMailBox_t FIFOMailBox[2];
    uint32_t  RESERVED1[12];
    volatile uint32_t FMR;
    volatile uint32_t FM1R;
    uint32_t  RESERVED2;
    volatile uint32_t FS1R;
    uint32_t  RESERVED3;
    volatile uint32_t FFA1R;
    uint32_t  RESERVED4;
    volatile uint32_t FA1R;
    uint32_t  RESERVED5[8];
    CAN_FilterRegister_t FilterBankRegister[28];

}CAN_Register_t;

#define CAN1 ((volatile CAN_Register_t *)CAN1_BASEADDR)
#define CAN2 ((volatile CAN_Register_t *)CAN2_BASEADDR)


 	 	 	 	 	 	 	 	 	 /*******TIMER*****/
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMCR;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t CCER;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
	volatile uint32_t CCR4;
	volatile uint32_t DCR;
	volatile uint32_t DMAR;
	volatile uint32_t OR;

}TIM_Register_t;
#define TIM1 (( volatile TIM_Register_t *)TIM1_BASEADDR)
#define TIM2 (( volatile TIM_Register_t *)TIM2_BASEADDR)
#define TIM3 (( volatile TIM_Register_t *)TIM3_BASEADDR)
#define TIM4 (( volatile TIM_Register_t *)TIM4_BASEADDR)
#define TIM5 (( volatile TIM_Register_t *)TIM5_BASEADDR)
#define TIM6 (( volatile TIM_Register_t *)TIM6_BASEADDR)
#define TIM7 (( volatile TIM_Register_t *)TIM7_BASEADDR)
#define TIM8 (( volatile TIM_Register_t *)TIM8_BASEADDR)
#define TIM9 (( volatile TIM_Register_t *)TIM9_BASEADDR)
#define TIM10 (( volatile TIM_Register_t *)TIM10_BASEADDR)
#define TIM11 (( volatile TIM_Register_t *)TIM11_BASEADDR)
#define TIM12 (( volatile TIM_Register_t *)TIM12_BASEADDR)
#define TIM13 (( volatile TIM_Register_t *)TIM13_BASEADDR)
#define TIM14 (( volatile TIM_Register_t *)TIM14_BASEADDR)



/*Clock enable and disable for GPIO */
/*Enable Begin*/
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1<<8))
#define GPIOJ_PCLK_EN() (RCC->AHB1ENR |= (1<<9))
#define GPIOK_PCLK_EN() (RCC->AHB1ENR |= (1<<10))
/*Enable End*/

/*Disable Begin*/
#define GPIOA_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<0))
#define GPIOB_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<1))
#define GPIOC_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<2))
#define GPIOD_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<3))
#define GPIOE_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<4))
#define GPIOF_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<5))
#define GPIOG_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<6))
#define GPIOH_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<7))
#define GPIOI_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<8))
#define GPIOJ_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<9))
#define GPIOK_PCLK_DIS() (RCC->AHB1ENR &=~ (1<<10))
/*Disable End*/

#define GPIOA_RESET()  do{ (RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &=~(1<<0));} while(0)
#define GPIOB_RESET()  do{ (RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &=~(1<<1));} while(0)
#define GPIOC_RESET()  do{ (RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &=~(1<<2));} while(0)
#define GPIOD_RESET()  do{ (RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &=~(1<<3));} while(0)
#define GPIOE_RESET()  do{ (RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &=~(1<<4));} while(0)
#define GPIOF_RESET()  do{ (RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &=~(1<<5));} while(0)
#define GPIOG_RESET()  do{ (RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &=~(1<<6));} while(0)
#define GPIOH_RESET()  do{ (RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &=~(1<<7));} while(0)
#define GPIOI_RESET()  do{ (RCC->AHB1RSTR |=(1<<8)); (RCC->AHB1RSTR &=~(1<<8));} while(0)
#define GPIOJ_RESET()  do{ (RCC->AHB1RSTR |=(1<<9)); (RCC->AHB1RSTR &=~(1<<9));} while(0)
#define GPIOK_RESET()  do{ (RCC->AHB1RSTR |=(1<<10)); (RCC->AHB1RSTR &=~(1<<10));} while(0)

#define IRQ_EXTI0 		6
#define IRQ_EXTI1 		7
#define IRQ_EXTI2 		8
#define IRQ_EXTI3 		9
#define IRQ_EXTI4 		10
#define IRQ_EXTI9_5 	23
#define IRQ_EXTI15_10 	40

#define IRQ_USART1	37
#define IRQ_USART2	38
#define IRQ_USART3	39
#define IRQ_UART4	52
#define IRQ_UART5	53
#define IRQ_USART6	71
#define IRQ_UART7	82
#define IRQ_UART8	83

#define IRQ_TIM9	24
#define IRQ_TIM10	25
#define IRQ_TIM2	28
#define IRQ_TIM3	29
#define IRQ_TIM4	30
#define IRQ_TIM5	50
#define IRQ_TIM6	54
#define IRQ_TIM7	55


/***********/

/*Clock enable and disable for UART*/
/*Enable Begin*/
#define USART1_PCLK_EN() (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |=(1<<18))
#define UART4_PCLK_EN()	 (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN()  (RCC->APB1ENR |=(1<<20))
#define USART6_PCLK_EN() (RCC->APB2ENR |=(1<<5))
#define UART7_PCLK_EN() (RCC->APB1ENR |=(1<<30))
#define UART8_PCLK_EN() (RCC->APB1ENR |=(1<<31))
/*Enable End*/

/*Disable Begin*/
#define USART1_PCLK_DIS() (RCC->APB2ENR &=~(1<<4))
#define USART2_PCLK_DIS() (RCC->APB1ENR &=~(1<<17))
#define USART3_PCLK_DIS() (RCC->APB1ENR &=~(1<<18))
#define UART4_PCLK_DIS()  (RCC->APB1ENR &=~(1<<19))
#define UART5_PCLK_DIS()  (RCC->APB1ENR &=~(1<<20))
#define USART6_PCLK_DIS() (RCC->APB2ENR &=~(1<<5))
#define UART7_PCLK_DIS()  (RCC->APB1ENR &=~(1<<30))
#define UART8_PCLK_DIS()  (RCC->APB1ENR &=~(1<<31))
/*Disable End*/

/*Clock enable and disable  for I2C*/
#define I2C1_PCLK_EN()	(RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |=(1<<23))

#define I2C1_PCLK_DIS()	(RCC->APB1ENR &=~(1<<21))
#define I2C2_PCLK_DIS()	(RCC->APB1ENR &=~(1<<22))
#define I2C3_PCLK_DIS()	(RCC->APB1ENR &=~(1<<23))

/*****************/

/*Clock enable and disable  for TIM*/
#define TIM2_PCLK_EN()	(RCC->APB1ENR |= (1<<0))
#define TIM3_PCLK_EN()	(RCC->APB1ENR |= (1<<1))
#define TIM4_PCLK_EN()	(RCC->APB1ENR |= (1<<2))
#define TIM5_PCLK_EN()	(RCC->APB1ENR |= (1<<3))
#define TIM6_PCLK_EN()	(RCC->APB1ENR |= (1<<4))
#define TIM7_PCLK_EN()	(RCC->APB1ENR |= (1<<5))
#define TIM12_PCLK_EN()	(RCC->APB1ENR |= (1<<6))
#define TIM13_PCLK_EN()	(RCC->APB1ENR |= (1<<7))
#define TIM14_PCLK_EN()	(RCC->APB1ENR |= (1<<8))
#define TIM1_PCLK_EN()	(RCC->APB2ENR |= (1<<0))
#define TIM8_PCLK_EN()	(RCC->APB2ENR |= (1<<1))
#define TIM9_PCLK_EN()	(RCC->APB2ENR |= (1<<16))
#define TIM10_PCLK_EN()	(RCC->APB2ENR |= (1<<17))
#define TIM11_PCLK_EN()	(RCC->APB2ENR |= (1<<18))


#define CAN1_PCLK_EN() 	(RCC->APB1ENR |= (1<<25))
#define CAN2_PCLK_EN() 	(RCC->APB1ENR |= (1<<25))

/*Clock enable and disable for SYSCFG*/
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |=(1<<14))
#define SYSCFG_PCLK_DIS() (RCC->APB2ENR & =~(1<<14))

/*Define status*/
#define ENABLE 1
#define DISABLE 0
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define FLAG_SET 1
#define FLAG_RESET 0
#define SET 1
#define RESET 0
/********/

#define GETBIT  0x1

#endif /*INC_STM32F4XX_H_ */
