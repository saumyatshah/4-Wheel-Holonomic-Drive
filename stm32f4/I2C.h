#include <stm32f407xx.h>
#include "stm32f407xx_1.h"
#ifndef _I2C_H
#define _I2C_H
//I2C Clock Enable Macros

#define I2C1_CLK_ENABLE  RCC->APB1ENR|=(1<<21)
#define I2C2_CLK_ENABLE  RCC->APB1ENR|=(1<<22)
#define I2C3_CLK_ENABLE  RCC->APB1ENR|=(1<<23)

//I2C Enable Macros
#define I2C1_ENABLE      I2C1->CR1|=(1<<0);
#define I2C2_ENABLE      I2C2->CR1|=(1<<0);
#define I2C3_ENABLE      I2C3->CR1|=(1<<0);

//I2C Speed Options
#define I2C_100K 1
#define I2C_400K 4

//I2C Ack control
#define I2C_ACK_ENABLE  1
#define I2C_ACK_DISABLE 0

//I2C FM Duty Cycle
#define I2C_DUTY_2      0
#define I2C_DUTY_16_9   1

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKcontrol;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;
	
typedef struct
{
	I2C_TypeDef *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTXBuffer;
	uint8_t *pRXBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t DevAdder;
	uint32_t RxSize;
	uint8_t sr;
}I2C_Handle_t;

/*
*   I2C Peripheral Clock Functions
*/
void I2C_Periclock_control(I2C_TypeDef *pI2Cx,uint8_t ENorDI);
/*
*   I2C Init and Deinit Functions
*/
void I2C_Init(I2C_Handle_t *pI2Chandle);
void I2C_DeInit(I2C_TypeDef *pI2Cx);
uint32_t Pclk();
/*
*   I2C Master and Slave Functions
*/
void I2C_MasterSenddata(I2C_TypeDef *pI2Chandle,uint8_t *pTxBuffer,uint8_t LEN,uint8_t ADDR);
void I2C_MasterReceivedata(I2C_TypeDef *pI2Chandle,uint8_t *pRxBuffer,uint8_t LEN,uint8_t ADDR,uint8_t RS);
void I2C_SlaveSenddata(I2C_TypeDef *pI2Chandle,uint8_t data);
uint8_t I2C_SlaveReceivedata(I2C_TypeDef *pI2Chandle);
/*
*   I2C Master and Slave Functions Wit Interrupt
*/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2Chandle,uint8_t *pTxBuffer,uint8_t LEN,uint8_t ADDR,uint8_t Sr);
uint8_t I2C_MasterReceivedataIT(I2C_TypeDef *pI2Chandle,uint8_t *pRxBuffer,uint8_t LEN,uint8_t ADDR);


/*
*   I2C IRQ Configuration and Handler Functions
*/
void I2C_IRQconfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQPriority(uint32_t IRQprio,uint8_t IRQNum);
void I2C_EV_IRQ(I2C_Handle_t *pI2Chandle);
void I2C_EV_IRQ(I2C_Handle_t *pI2Chandle);













#endif
