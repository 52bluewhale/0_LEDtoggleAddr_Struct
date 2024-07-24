/*to turn on the LED, the question is
 * Where is the LED connected?
 * Port:	A
 * Pin:		5
*/
#include <stdint.h>
#define PERIPH_BASE				(0x40000000UL)

#define AHB1_PERIPH_OFFSET		(0x00020000UL)
#define AHB1_PERIPH_BASE		(PERIPH_BASE + AHB1_PERIPH_OFFSET)

#define GPIO_A_OFFSET			(0x0000UL)
#define GPIO_A_BASE				(AHB1_PERIPH_BASE + GPIO_A_OFFSET)

#define RCC_OFFSET				(0x3800UL)
#define RCC_BASE				(AHB1_PERIPH_BASE + RCC_OFFSET)

//#define AHB1_EN_R_OFFSET		(0x30UL)
//#define RCC_AHB1_EN_R			(*(volatile unsigned int *)(RCC_BASE + AHB1_EN_R_OFFSET))

#define GPIO_A_EN				(1U<<0)	//0x0000 0000 0000 0000 0000 0000 0000 0001

//#define MODE_R_OFFSET			(0x00UL)
//#define GPIO_A_MODE_R			(*(volatile unsigned int *)(GPIO_A_BASE + MODE_R_OFFSET))
//
//#define OD_R_OFFSET				(0x14UL)
//#define GPIO_A_OD_R				(*(volatile unsigned int *)(GPIO_A_BASE + OD_R_OFFSET))

#define PIN_5					(1U<<5)
#define LED_PIN					PIN_5

#define __IO					volatile

typedef struct
{
	__IO uint32_t MODER;
	volatile uint32_t DUMMY[4];
	__IO uint32_t ODR;
}GPIO_Typedef;

//typedef struct
//{
//	__IO uint32_t MODER;	/*!< GPIO port mode register, 				Address offset: 0x00		*/
//	__IO uint32_t OTYPER;	/*!< GPIO port output type register,		Address offset: 0x04		*/
//	__IO uint32_t OSPEEDR;	/*!< GPIO port output speed register,		Address offset: 0x08		*/
//	__IO uint32_t PUPDR;	/*!< GPIO port pull-up/pull-down register,	Address offset: 0x0C		*/
//	__IO uint32_t IDR;		/*!< GPIO port input data register,			Address offset: 0x10		*/
//	__IO uint32_t ODR;		/*!< GPIO port output data register,		Address offset: 0x14		*/
//	__IO uint32_t BSRR;		/*!< GPIO port bit set/reset register,		Address offset: 0x18		*/
//	__IO uint32_t LCKR;		/*!< GPIO port configuration lock register,	Address offset: 0x1C		*/
//	__IO uint32_t AFR[2];	/*!< GPIO alternate function registers,		Address offset: 0x20 - 0x24	*/
//}GPIO_Typedef;

typedef struct
{
	volatile uint32_t DUMMY[12];
	__IO uint32_t AHB1ENR;
}RCC_Typedef;

//typedef struct
//{
//	__IO uint32_t CR;/*!< RCC clock control register,											Address offset: 0x00*/
//	__IO uint32_t PLLCFGR;/*!< RCC PLL configuration register,									Address offset: 0x04*/
//	__IO uint32_t CFGR;/*!< RCC clock configuration register,									Address offset: 0x08*/
//	__IO uint32_t CIR;/*!< RCC clock interrupt register,										Address offset: 0x0C*/
//	__IO uint32_t AHB1RSTR;/*!< RCC AHB1 peripheral reset register,								Address offset: 0x10*/
//	__IO uint32_t AHB2RSTR;/*!< RCC AHB2 peripheral reset register,								Address offset: 0x14*/
//	__IO uint32_t AHB3RSTR;/*!< RCC AHB3 peripheral reset register,								Address offset: 0x18*/
//	uint32_t 	  RESERVED0;/*!< Reserved,														Address offset: 0x1C*/
//	__IO uint32_t APB1RSTR;/*!< RCC APB1 peripheral reset register,								Address offset: 0x20*/
//	__IO uint32_t APB2RSTR;/*!< RCC APB2 peripheral reset register,								Address offset: 0x24*/
//	uint32_t      RESERVED1[2];/*!< Reserved,													Address offset: 0x28 - 0x2C*/
//	__IO uint32_t AHB1ENR;/*!< RCC AHB1 peripheral clock enable register,						Address offset: 0x30*/
//	__IO uint32_t AHB2ENR;/*!< RCC AHB2 peripheral clock enable register,						Address offset: 0x34*/
//	__IO uint32_t AHB3ENR;/*!< RCC AHB3 peripheral clock enable register,						Address offset: 0x38*/
//	uint32_t      RESERVED2;/*!< Reserved,														Address offset: 0x3C*/
//	__IO uint32_t APB1ENR;/*!< RCC APB1 peripheral clock enable register,						Address offset: 0x40*/
//	__IO uint32_t APB2ENR;/*!< RCC APB2 peripheral clock enable register,						Address offset: 0x44*/
//	uint32_t	  RESERVED3[2];/*!< Reserved													Address offset: 0x48 - 4C*/
//	__IO uint32_t AHB1LPENR;/*!< RCC AHB1 peripheral clock enable in low power mode register,	Address offset: 0x50*/
//	__IO uint32_t AHB2LPENR;/*!< RCC AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54*/
//	__IO uint32_t AHB3LPENR;/*!< RCC AHB3 peripheral clock enable in low power mode register,	Address offset: 0x58*/
//	uint32_t	  RESERVED4;/*!< Reserved,														Address offset: 0x5C*/
//	__IO uint32_t APB1LPENR;/*!< RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60*/
//	__IO uint32_t APB2LPENR;/*!< RCC APB2 peripheral clock enable in low power mode register,	Address offset: 0x64*/
//	uint32_t	  RESERVED5[2];/*!< Reserved,													Address offset: 0x68 - 6C*/
//	__IO uint32_t BDCR;/*!< RCC backup domain control register									Address offset: 0x70*/
//	__IO uint32_t CSR;/*!< RCC clock control & status register,									Address offset: 0x74*/
//	uint32_t	  RESERVED6[2];/*!< Reserved,													Address offset: 0x78 - 7C*/
//	__IO uint32_t SSCGR;/*!< RCC spread spectrum clock generation register,						Address offset: 0x80*/
//	__IO uint32_t PLLI2SCFGR;/*!< RCC PLLI2S configuration register,							Address offset: 0x84*/
//	uint32_t	  RESERVED7[1];/*!< Reserved,													Address offset: 0x88*/
//	__IO uint32_t DCKCFGR;/*!< RCC dedicated clocks configuration register,						Address offset: 0x8C*/
//}RCC_Typedef;


/*
 * (1U<<10) >>>>>>set bit 10 to 1
 * &=~(1U<<11) >>>set bit 11 to 0
 */

/*
 * avoid FRIENDLY FIRE while configuring the RCC_AHB1_EN_R:
 * what if we already had the content of the RCC_AHB1_EN_R like this
 *
 * RCC_AHB1_EN_R = 0x0000 0000 0000 0000 1000 1111 0000 0000
 *
 * and we only want to change the value of bit[0] of the register to 1
 * if we said that:
 * 		RCC_AHB1_EN_R = GPIO_A_EN;
 * it will clear all the content of the register
 *
 * so, we have to use OR operator in order to remain the content of the register:
 * 		RCC_AHB1_EN_R |= GPIO_A_EN;
 *
 * */

#define RCC 					((RCC_Typedef*) RCC_BASE)
#define GPIO_A 					((GPIO_Typedef*) GPIO_A_BASE)

int main(void)
{
	/*1. Enable clock for GPIO A*/
	//RCC_AHB1_EN_R |= GPIO_A_EN;
	RCC->AHB1ENR |= GPIO_A_EN;

	/*2. Set PA5 as output pin*/
	//GPIO_A_MODE_R |= (1U<<10);
	//GPIO_A_MODE_R &= ~(1U<<11);
	GPIO_A->MODER |= (1U<<10);
	GPIO_A->MODER &= ~(1U<<11);

	while(1)
	{
		/*3. Set PA5 high*/
		//GPIO_A_OD_R |= LED_PIN;
		GPIO_A->ODR ^= LED_PIN;

		/*4. Toggle PA5*/
		//GPIO_A_OD_R ^= LED_PIN;
		GPIO_A->ODR ^= LED_PIN;
		for (int i=0; i<500000; ++i)
		{

		}
	}
}
