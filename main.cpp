#include "stm32f4xx.h"                  // Device header
#include "stm32f4/common.h"
#include <math.h>
int main(void)
{
	////////////////////////////////////////////////PLL///////////////////////////////////////////////
	RCC_Init();	
	jtag();
	///////////////////////////////////////////////DELAY CONFIG//////////////////////////////////////
	delay_con();//tim12
	//////////////////////////////////////////////LCD/////////////////////////////////////////////////
	ConfigLcd();
	//COM0();
	COM1();
	cls();
	int k;
	lcd((char*)"TeaM InDiA");
	lowerline();
	lcd((char*)"ROBOCON 2020");	
	
/*	Spi_GPIO();
	SPI_Handle_t SPI_Handle;
	SPI_ClockControl(SPI1,ENABLE);
  SPI_Handle.pSPI=SPI1;
	
	SPI_Handle.SPIConfig.CPHA=CPHA_0;
	SPI_Handle.SPIConfig.CPOL=CPOL_0;
	SPI_Handle.SPIConfig.SSM=SSM_DI;
	SPI_Handle.SPIConfig.FF=MSB;
  SPI_Handle.SPIConfig.DeviceMode=SPI_MASTER;
	SPI_Handle.SPIConfig.BusConfig=Full_Duplex;
	SPI_Handle.SPIConfig.DFF=SPI_8_BIT;
	SPI_Handle.SPIConfig.Frequency=SPI_F_32;
  SPI_Init(&SPI_Handle);
  //IMU_TIMER_CONF();    
	
  TIMER6_ENABLE;
//int k[4];
while(1)
{
	 GPIO_Write(GPIOA,GPIO_4,RESET);
	SPI_Communicate(SPI1,0x80);
  //SPI_Communicate(SPI2,100);
	GPIO_Write(GPIOA,GPIO_4,SET);
	
}*/
	
	while(1)
	{
	//	k=USART_ReceiveData(USART1);
		USART_SendData(USART1,123);
		
	}
	return 0;

}
