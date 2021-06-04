# stm32-RCC-configuration-of-clock-system
stm32时钟系统RCC配置
code:
---
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"

#define XTAl	
#define HSE_STARTUP_TIMEOUT ((uint16_t)0xFFFF)

#define	AHB_PRE			1
#define APB1_PRE		2
#define APB2_PRE		1
#define SysTicksClk 10000

#define	SYSCLK			72000000
#define	AHB					SYSCLK/AHB_PRE//72
#define APB1				AHB/APB1_PRE//36
#define APB1_TIM		APB1*2
#define APB2				AHB/APB2_PRE//72
#define APB2_TIM		APB2*1
#define SysTicks		AHB/SysTicksClk

void ChangeLED(unsigned counter);
void SysTick_Handler(void);



int main()
{
	unsigned counter=0x0000;
	__IO uint32_t StartUpCounter= 0,HSEStatus=0;
	
	//Enable HSE
	//if HSI ready,config system clocks
	if ((RCC->CR & RCC_CR_HSIRDY) != RESET) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |RCC_AHB1ENR_GPIOBEN |RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= 1 << 2 * 2;// Set PD2 to output mode
	// Enable FLASH Prefetch
	FLASH->ACR |= FLASH_ACR_PRFTEN;//Prefetch enable
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;//Latency
		//RCC clock configuration
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHBdiv1=72
	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;//AHB1div2=36
	RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;//AHB2div1=72
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
	RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;//VCOinput=PLLinput/PLLM=16/8=2  BestValue,which is recommended value
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
	RCC->PLLCFGR |= 72 << RCC_PLLCFGR_PLLN_Pos;//VCOoutput=VCOinput*PLLN=2*36=72;but need div2;	36*Latency=36*2=72
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk;
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	// Set SysClk from PLL
	RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));/*!< SW[1:0] bits (System clock Switch) */
	RCC->CFGR |= RCC_CFGR_SW_PLL;/*!< PLL selected as system clock */
	}
	SysTick_Config(SysTicks);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	GPIOA->MODER |=0x00005555;
	GPIOD->MODER |=1<<(2*2);
	while(1)
	{
		if (!(GPIOB->IDR & GPIO_IDR_ID0))
		{
			counter=counter+1;
			while((!(GPIOB->IDR & GPIO_IDR_ID0)));
			ChangeLED(counter);
		}
		else if (!(GPIOB->IDR & GPIO_IDR_ID1))
		{
			counter=counter-1;
			while((!(GPIOB->IDR & GPIO_IDR_ID1)));
			ChangeLED(counter);
		}		
	}
}

void ChangeLED(unsigned counter)
{
	GPIOA->ODR |= counter;
	GPIOA->ODR &= counter;
}

void SysTick_Handler(void)
{
	if (GPIOD->ODR & GPIO_ODR_OD2)
	{
		GPIOD->BSRR |= GPIO_BSRR_BR2;
	}
	else
	{
		GPIOD->BSRR |= GPIO_BSRR_BS2;
	}
}

```
proreus:
---
![0001](https://user-images.githubusercontent.com/74950715/120735650-9174b380-c51d-11eb-8016-c5eb8533601c.png)

