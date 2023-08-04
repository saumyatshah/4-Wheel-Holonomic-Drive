#include "I2C.h"
/*
*Basic Instruction To Operate I2C Peripheral
*NEVER USE RANDOM PULL-UP RESISTANCE FOR I2C
*USE FORMULA FOR PULL-UP RESISTANCE
*/


void I2C_Periclock_control(I2C_TypeDef *pI2Cx,uint8_t ENorDI)
{
	if(ENorDI==ENABLE)
	{
		switch((int)pI2Cx)
		{
			case (int)I2C1:
		  I2C1_CLK_ENABLE;
			break;
			
			case (int)I2C2:
			I2C2_CLK_ENABLE;	
			break;
			
			case (int)I2C3:
			I2C3_CLK_ENABLE;	
			break;
		}
	}
}
uint32_t Pclk()
{
	uint8_t temp=0;
	uint8_t temp1=0;
	uint8_t temp2=0;
	uint8_t PCLK;
	uint8_t CLK;
	uint8_t AHB_PCLK;
	uint8_t APB_PCLK;
	uint16_t AHB_CLK[]={0,2,4,8,16,32,64,128,256,512};
	uint8_t  APB_CLK[]={0,4,5,6,7};
	temp=((RCC->CFGR>>2) & 0x03);
	temp1=((RCC->CFGR>>4) & 0xF);
	temp2=((RCC->CFGR>>10) & 0x7);
	//Ceck te system clock
	if(temp==0)
		CLK=16;
	else if(temp==1)
	  CLK=8;
	else if(temp==2)
		CLK=168;
	//Ceck te clock of AB Bus
	AHB_PCLK=AHB_CLK[temp1-8];
	//Ceck te clock of APB Bus
	APB_PCLK=APB_CLK[temp2-4];
	//Calculation of FREQ
	PCLK=(CLK/AHB_PCLK)/APB_PCLK;
	
	return PCLK;
	
}
void I2C_Init(I2C_Handle_t *pI2Chandle)
{
	   
	   //Enable clock
	  I2C_Periclock_control(pI2Chandle->pI2Cx,ENABLE);
	  I2C1_ENABLE;
	 
	  //ACK bit enable
    pI2Chandle->pI2Cx->CR1|=(1<<10);	//pI2Chandle->I2C_Config.I2C_ACKcontrol
		
		//FREQ Configuration
	  pI2Chandle->pI2Cx->CR2|=16;//(Pclk() & 0x3F);
	  
	  //CCR Configuration
	   if(pI2Chandle->I2C_Config.I2C_SCLSpeed<=I2C_100K)
		{
  		pI2Chandle->pI2Cx->CCR|=(0<<15);
      pI2Chandle->pI2Cx->CCR|=0x50;//(Pclk()/(pI2Chandle->I2C_Config.I2C_SCLSpeed*2));
		}	
    else
		{
			pI2Chandle->pI2Cx->CCR|=(1<<15);// Fast and Slow mode Configuration 
			pI2Chandle->pI2Cx->CCR|=(pI2Chandle->I2C_Config.I2C_FMDutyCycle<<14);//Duty Cycle of I2C
			
			if(pI2Chandle->I2C_Config.I2C_FMDutyCycle== I2C_DUTY_2)
				pI2Chandle->pI2Cx->CCR|=(Pclk()/(pI2Chandle->I2C_Config.I2C_SCLSpeed*3));
			else
        pI2Chandle->pI2Cx->CCR|=(Pclk()/(pI2Chandle->I2C_Config.I2C_SCLSpeed*25));
		}
		//TRISE Configuration(MOST IMP)
		if(pI2Chandle->I2C_Config.I2C_SCLSpeed<=I2C_100K)
			pI2Chandle->pI2Cx->TRISE|=0x11;//(Pclk()+1);
		else
			pI2Chandle->pI2Cx->TRISE|=((3*Pclk())/10)+1;
		
	  //ADDR Configuration
	  pI2Chandle->pI2Cx->OAR1=(pI2Chandle->I2C_Config.I2C_DeviceAddress<<1)|(1<<14);
	  //Timeout
		
	
}

void I2C_MasterSenddata(I2C_TypeDef *pI2Chandle,uint8_t *pTxBuffer,uint8_t LEN,uint8_t ADDR)
{
	//Follow and Write code same as ardware workin in I2C
	 uint32_t dummy;

	//Generate Start Condition 
	pI2Chandle->CR1|=(1<<8);
	//Wait untill start condition transmitted
	while(!((pI2Chandle->SR1) & 0x01));
	dummy=pI2Chandle->SR1;
	//Send slave address
	ADDR=ADDR<<1;
	//ADDR&=~(0x01);//ADDR + R/W BIT 
	pI2Chandle->DR=ADDR;
	
	//Ceck weater addr is transmitted or not
	 while(!((pI2Chandle->SR1>>1)&0x01));
   dummy=pI2Chandle->SR1;//Dummy Readin	
	//Send data slave
	while(LEN>0)
	{
		
		pI2Chandle->DR=*pTxBuffer;
		pTxBuffer++;
		LEN--;
		while(!((pI2Chandle->SR1>>7)&0x01));
	}
	pI2Chandle->CR1|=(1<<9);
	//For stop condition BTF and TX bot fla sould be 1
	//while(!((pI2Chandle->SR1>>7)&0x01));//TX Fla
	//while(!((pI2Chandle->SR1>>2)&0x01));//BTF Fla
	//STOP condition
	
	
	//END
	
}

void I2C_MasterReceivedata(I2C_TypeDef *pI2Chandle,uint8_t *pRxBuffer,uint8_t LEN,uint8_t ADDR,uint8_t RS)
{
	//Start Condition
	pI2Chandle->CR1|=(1<<8);
	//Wait untill start condition transmitted
	while(!((pI2Chandle->SR1) & 0x01));
	(void)pI2Chandle->SR1;
	//Send slave address
	ADDR=ADDR<<1;
	ADDR=ADDR|1;
	//ADDR&=~(0x01);//ADDR + R/W BIT 
	pI2Chandle->DR=ADDR;
	
	//Ceck weater addr is transmitted or not
	 while(!((pI2Chandle->SR1>>1)&0x01));
   (void)pI2Chandle->SR1;//Dummy Readin	
	
	 if(LEN==1)
	 {
		 //Disable Ack
		 pI2Chandle->CR1&=~(1<<10);
		 //Clear ADDR Fla
		 pI2Chandle->SR1&=~(1<<1);
		 //Wait untill RXNE=1 
	   while(!((pI2Chandle->SR1>>6) & 0x01));
		 //Stop Condition
		 pI2Chandle->CR1|=(1<<9);
		 //Readin operation
		 *pRxBuffer=pI2Chandle->DR;
		 
	 }
	 if(LEN>>1)
	 {
		 //Clear addr fla
		 pI2Chandle->SR1&=~(1<<1);
		  
		    for(uint32_t i=LEN;i>0;i--)
		      {
						//Wait untill RXNE=1
						 while(!((pI2Chandle->SR1>>6) & 0x01));
						   if(i==2)
							 {
								 //Disable ack
								 pI2Chandle->CR1&=~(1<<10);
								 //Stop Condition
							   pI2Chandle->CR1|=(1<<9);
							 }
						//Data 
						*pRxBuffer=pI2Chandle->DR;
						//Increment Buffer
						pRxBuffer++;
							 
					}
		 
	 }
	 //Enable ack
	 pI2Chandle->CR1|=(1<<10);
}

/*uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTXBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = ;
		pI2CHandle->DevAdder = SlaveAddr;
		pI2CHandle->sr =Sr;

		//Implement code to Generate START Condition
		

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		

		//Implement the code to enable ITERREN Control Bit
		

	}

	return busystate;

}*/
void I2C_SlaveSenddata(I2C_TypeDef *pI2Chandle,uint8_t data)
{
	pI2Chandle->DR=data;
}

uint8_t I2C_SlaveReceivedata(I2C_TypeDef *pI2Chandle)
{
	return (uint8_t)pI2Chandle->DR;
}
void I2C_EV_IRQ(I2C_Handle_t *pI2Chandle)
{
	
	
}

