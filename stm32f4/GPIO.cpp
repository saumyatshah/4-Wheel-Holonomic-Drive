#include "stm32f407xx_1.h"
#include <stm32f407xx.h>
#include <core_cm4.h>
#include "GPIO.h"
#define SYSCFG_CLK_EN      RCC_APB2ENR|=(1<<14)

void GPIO_Periclock_control(GPIO_TypeDef *pGPIOx,uint8_t ENorDI)
{
	if(ENorDI==ENABLE)
	{
		switch((int)pGPIOx)
		{
			case (int)GPIOA:
			GPIOA_PCLK_EN;
			break;
			
			case (int)GPIOB:
			GPIOB_PCLK_EN;	
			break;
			
			case (int)GPIOC:
			GPIOC_PCLK_EN;	
			break;
			
			case (int)GPIOD:
			GPIOD_PCLK_EN;	
			break;
			
			case (int)GPIOE:
			GPIOE_PCLK_EN;	
			break;
			
			case (int)GPIOF:
			GPIOF_PCLK_EN;	
			break;
			
			case (int)GPIOG:
			GPIOG_PCLK_EN;	
			break;
			
			case (int)GPIOH:
			GPIOH_PCLK_EN;	
			break;
			
			case (int)GPIOI:
			GPIOI_PCLK_EN;	
			break;
		}
	}
	else
	{
		switch((int)pGPIOx)
		{
			case (int)GPIOA:
			GPIOA_PCLK_DI;
			break;
			
			case (int)GPIOB:
			GPIOB_PCLK_DI;	
			break;
			
			case (int)GPIOC:
			GPIOC_PCLK_DI;	
			break;
			
			case (int)GPIOD:
			GPIOD_PCLK_DI;	
			break;
			
			case (int)GPIOE:
			GPIOE_PCLK_DI;	
			break;
			
			case (int)GPIOF:
			GPIOF_PCLK_DI;	
			break;
			
			case (int)GPIOG:
			GPIOG_PCLK_DI;	
			break;
			
			case (int)GPIOH:
			GPIOH_PCLK_DI;	
			break;
			
			case (int)GPIOI:
			GPIOI_PCLK_DI;	
			break;
		}
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_Periclock_control(pGPIOHandle->pGPIOx,ENABLE);
	uint32_t temp;
	temp=0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=3)
  {
		//Input & Output Configuration
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//pGPIOHandle->pGPIOx->MODER&=~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pins
		pGPIOHandle->pGPIOx->MODER|=temp;                                                //setting pins
	}
  else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==4)
		{
	    EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==5)
		{
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==6)
		{
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		
		SYSCFG_CLK_EN;
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=3)
		{
			SYSCFG->EXTICR[0]|=(pGPIOHandle->GPIO_PinConfig.GPIO_IntPort<<4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=7 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>3)
		{
		  SYSCFG->EXTICR[1]|=(pGPIOHandle->GPIO_PinConfig.GPIO_IntPort<<4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-4));
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=11 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>7)
		{
			SYSCFG->EXTICR[2]|=(pGPIOHandle->GPIO_PinConfig.GPIO_IntPort<<4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8));
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=15 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>11)
		{
		  SYSCFG->EXTICR[3]|=(pGPIOHandle->GPIO_PinConfig.GPIO_IntPort<<4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-12));
		}
		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		
}
		//Output Switching Configuration
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR&=~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pins
		pGPIOHandle->pGPIOx->OSPEEDR|=temp;
		//Pull-up & Pull-down Configuration
	  temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->PUPDR&=~(0x03<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pins
		pGPIOHandle->pGPIOx->PUPDR|=temp;
	  //Output type pus-pull & open-drain 
	  temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinOPtype << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		pGPIOHandle->pGPIOx->OTYPER&=~(0x01<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pins
		pGPIOHandle->pGPIOx->OTYPER|=temp;
		//Alternate function Configuration
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALE)
	  { 
		    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<8)
				 { 
					 temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
					 //pGPIOHandle->pGPIOx->AFR[0]&=~(0xF<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear pins
					 pGPIOHandle->pGPIOx->AFR[0]|=temp;
				 }
				else
				{ 
					temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8));
				  //pGPIOHandle->pGPIOx->AFR[1]&=~(0xF<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8)); //clear pins
					pGPIOHandle->pGPIOx->AFR[1]|=temp;
				}
		}
}

void GPIO_DeInit(GPIO_TypeDef *pGPIOx)
{
	switch((int)pGPIOx)
		{
			case (int)GPIOA:
			GPIOA_RESET;
			break;
			
			case (int)GPIOB:
			GPIOB_RESET;	
			break;
			
			case (int)GPIOC:
			GPIOC_RESET;	
			break;
			
			case (int)GPIOD:
			GPIOD_RESET;	
			break;
			
			case (int)GPIOE:
			GPIOE_RESET;	
			break;
			
			case (int)GPIOF:
			GPIOF_RESET;	
			break;
			
			case (int)GPIOG:
			GPIOG_RESET;	
			break;
			
			case (int)GPIOH:
			GPIOH_RESET;	
			break;
			
			case (int)GPIOI:
			GPIOI_RESET;	
			break;
		}
}

uint8_t GPIO_Read(GPIO_TypeDef *pGPIOx,uint8_t PinNumber)
{
	uint16_t value=0;
	value=pGPIOx->IDR;
	value=(uint8_t)((value>>PinNumber)&(0x01));
	return value;
}

void GPIO_Write(GPIO_TypeDef *pGPIOx,uint8_t Pinnumber,uint8_t Value)
{
	if(Value==SET)
	 pGPIOx->ODR|=(1<<Pinnumber);
	else
		pGPIOx->ODR&=~(1<<Pinnumber);
}

void GPIO_Toggle(GPIO_TypeDef *pGPIOx,uint8_t PinNumber)
{
	pGPIOx->ODR^=(1<<PinNumber);
}

void GPIO_IRQconfig(uint8_t IRQNumber,uint8_t EnorDi)
{
    if(EnorDi== ENABLE)
		{
			   if(IRQNumber<=31)
		      *NVIC_ISER0|=(1<<IRQNumber);
				 else if(IRQNumber >31 && IRQNumber<64)
				  *NVIC_ISER1|=(1<<(IRQNumber-32));
         else if(IRQNumber>=64 && IRQNumber<96)
				  *NVIC_ISER3|=(1<<(IRQNumber-64));
		}
		else
		{
			   if(IRQNumber<=31)
				  *NVIC_ICER0|=(1<<6);
				 else if(IRQNumber >31 && IRQNumber<64)
				  *NVIC_ICER1|=(1<<(IRQNumber-32));
         else if(IRQNumber>=64 && IRQNumber<96)
				  *NVIC_ICER2|=(1<<(IRQNumber-64));
		}
}

void GPIO_IRQPriority(uint32_t IRQprio,uint8_t IRQNum)
{
	uint8_t N=IRQNum/4;
	uint8_t U=IRQNum%4;
	uint8_t sift=8*U+(8-4);
	*(NVIC_IPR_BASE+N)|=((IRQprio)<<(sift));
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	 if(EXTI->PR & (1<<PinNumber))
	 {EXTI->PR|=(1<<PinNumber);}
}
/*//Example code for GPIO Configuration 
//Here portd is used and interrup is enabled on EXTI9_5 line
  GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOD;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_5;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_FT; //falling edge
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PUUP;
	GPIO_Btn.GPIO_PinConfig.GPIO_IntPort=PD;
	GPIO_Init(&GPIO_Btn);
	GPIO_Periclock_control(GPIOD,ENABLE);
	GPIO_IRQPriority(0,EXTI9_5_IRQn);
	GPIO_IRQconfig(EXTI9_5_IRQn,ENABLE);
*/
