#include "common.h"
#include <math.h>


////////////////////////////////////////////////PS4 VARIABLES////////////////////////////////////////////////////////////////
bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2,ps_touchpad,ps_l3,goti_drive;
int ps2_val[4], one=0,two=0,three=0,four=0,turn_adc=0,turn_yes=0,top_gate=0,bottom_gate=0,top_gate_2=0;
uint8_t left_joy=0,right_joy=0;
double temp_four=0;          ///// temp_four1 orignally not declared here but was  declared by me 
int turn_final,major_speed=0,minor_speed=0;///// temp_four1 orignally not declared here but was  declared by me 
bool kick_count,kapda_count,flap_count;
int ps2_minor_speed=0,ps2_major_speed=0,countcycle=0,countcycle1=0,imucounter=0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double drive_ang;
////////////////////////////////////////////////IMU VARIABLES////////////////////////////////////////////////////////////////
double ang=0,ans1=0;
uint8_t res[4];
bool imu_con=0;
int ans=0;
double count;



// line sensor + line sensor PID
		
double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
int sum_sensor_v=0,sum_sensor_h=0;
bool sensor_value[8];
bool h_plus_sensor[16],v_plus_sensor[16];           /////  FOR PLUS SENSOR 	
double error_v_plus[7],net_error_v_plus,error_h_plus[7],net_error_h_plus=0,left_junction=0,right_junction=0,right_junction_flag=0,left_junction_flag=0, full_junction=0,full_junction_flag=0;
double new_line_heading=0,new_kp_line=0;
int x_temp,y_temp;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//velocity feedback & pos
double x_dis=0,y_dis=0,x_d_dis,y_d_dis,prev_x_d,prev_y_d,th;
int16_t array_x[100],array_y[100],readings=0;
int i=0,j=0;
double x_vel,y_vel,r_vel,ang_vel,prev_l,prev_r,k=0,prev_x,prev_y,speed=0;
float Vx,Vy,kpv,kdv,kiv,prevd_x,prevd_y,integ_x,integ_y;
///syncro drive
double	m,d_x,a_x,a_y,d_y,diff_x,diff_y,dis,dr_ang,ang_d, coor_spd,base_spd;
int temp_x=0,temp_y=0,temp_x1=0,temp_x2=0,temp_x3=0,temp_y1=0,temp_y2=0,temp_y3=0;
bool v=0;
	int fl,fr,bl,br;
	double cur_tick=0,dif_ang, dif_tick,prev_error, integrld,temp_tick, syn_tick,icontrl,contrl,proprnl,difrnl,integrl;
		double s_p,s_d,s_i, s_ang;
		signed int rot_spd;
		int8_t invert=1,navin;
		unsigned short F_value;
		int16_t prev_syn_ang,syn_difference;
		double vel_error,speed_control,ispeed_control,f_speed;

////////////////////////////////////////////////DRIVE VARIABLES////////////////////////////////////////////////////////////////
double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
double lim11,lim12;             //limits for channel 1 -> with stop value 64    80,48
double lim21,lim22;           //limits for channel 2 -> with stop value 192   208,176
int lim,control_speed=20,flag=0,previous,prevposition;
double acc=0;
int base_value,tx_1,tx_2,tx_3,tx_4;
char dummyl,dummyr,txt1,txt2,txt3;
float kp=0,ki=0,kd=0;
double b_heading,hold_angle=0,value_1,value_2,value_3,value_4, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol=0,difference;

double Speedx,s_tick,s_p_tick,s_y_tick,s_y_p_tick,Speedy;
int counter,prv_counter,tick_diff;
double data[50];
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////DEAD RACON VARIABLES////////////////////////////////////////////////////////
double d_kp=0.8,d_kd,d_ki,d_error,ideal,dr_error,out;


/*
*------------------------------------------NVIC FUNCTION POINTER------------------------------------------- 
*/
void (*IMU_TIMER)();
void (*te)();

/////////////////////////////////////////////////////CONTROLLER SPECIFIC FUNCTION/////////////////////////////////////////////////////
void jtag()
{
	
	
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_13;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=0;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_14;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_15;
	GPIO_Init(&GPIO_Btn);
	
	GPIO_Btn.pGPIOx=GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_3;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=0;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_4;
	GPIO_Init(&GPIO_Btn);
	
}


//////////////////////////////////////////////////////////////JTAG END////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////ENCODER CONFIGURATION////////////////////////////////////////////////////////////////////////////

void ENCODER_GPIO_X()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_5;//PA5
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=1;
	GPIO_Init(&GPIO_Btn);

	GPIO_Btn.pGPIOx=GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_3;//PB3
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=1;
	GPIO_Init(&GPIO_Btn);
}
void ENCODER_GPIO_S()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_6;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=2;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_7;
	GPIO_Init(&GPIO_Btn);
}
void ENCODER_GPIO_Y()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=2;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_1;
	GPIO_Init(&GPIO_Btn);
}
void ENCODER()
{
	ENCODER_GPIO_X();
	TimerPeriClock(TIM2,ENABLE);
	Timer_Handle_t capturex;
	capturex.pTIMx=TIM2;
	capturex.Timer_Config.Mode=1;
	capturex.Timer_Config.CapEdge=RISING;
	capturex.Timer_Config.period=0xFFFFFFFE;
	Timer_Init(&capturex);
	TIMER2_ENABLE;
	
	
	ENCODER_GPIO_Y();
	TimerPeriClock(TIM5,ENABLE);
	Timer_Handle_t capturey;
	capturey.pTIMx=TIM5;
	capturey.Timer_Config.Mode=1;
	capturey.Timer_Config.CapEdge=RISING;
	capturey.Timer_Config.period=0xFFFFFFFE;
	Timer_Init(&capturey);
	TIMER5_ENABLE;
	
//	ENCODER_GPIO_S();
//	TimerPeriClock(TIM3,ENABLE);
//	Timer_Handle_t captures;
//	captures.pTIMx=TIM3;
//	captures.Timer_Config.Mode=1;
//	captures.Timer_Config.CapEdge=RISING;
//	captures.Timer_Config.period=0xFFFFFFFE;
//	Timer_Init(&captures);
//	TIMER3_ENABLE;
	
}



/////////////////////////////////////////////////////////////ENCODER END////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////ALL UART CONFIGURATION////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////COM1 CONFIGURATION////////////////////////////////////////////////////////////////////////////////

void COM1_GPIO()
{
	
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_9;//TX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_10;//RX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	
}

void COM1()
{
	
	COM1_GPIO();
	USART_Handle_t USART_Handle;
	USART_Handle.pUSARTx=USART1;
	USART_Handle.USART_Config.USART_Baud=BAUD_19200;
	USART_Handle.USART_Config.USART_Mode=_TXRX;
	USART_Handle.USART_Config.USART_NoOfStopBits=STOPBITS_1;
	USART_Handle.USART_Config.USART_ParityControl=PARITY_DISABLE;
	USART_Handle.USART_Config.USART_WordLength=USART_8BITS;
	USART_Init(&USART_Handle);
	
}

//////////////////////////////////////////////////COM2 CONFIGURATION////////////////////////////////////////////////////////////////////////////////

void COM2_GPIO()
{
	
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_2;//TX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_3;//RX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	
}

void COM2()
{
	
	COM2_GPIO();
	USART_Handle_t USART_Handle;
	USART_Handle.pUSARTx=USART2;
	USART_Handle.USART_Config.USART_Baud=BAUD_19200;
	USART_Handle.USART_Config.USART_Mode=_TXRX;
	USART_Handle.USART_Config.USART_NoOfStopBits=STOPBITS_1;
	USART_Handle.USART_Config.USART_ParityControl=PARITY_DISABLE;
	USART_Handle.USART_Config.USART_WordLength=USART_8BITS;
	USART_Init(&USART_Handle);
	
}


//////////////////////////////////////////////////COM3 CONFIGURATION////////////////////////////////////////////////////////////////////////////////

void COM3_GPIO()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_10;//TX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_11;//RX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
}

void COM3()
{
	COM3_GPIO();
	USART_Handle_t USART_Handle;
	USART_Handle.pUSARTx=USART3;
	USART_Handle.USART_Config.USART_Baud=BAUD_19200;
	USART_Handle.USART_Config.USART_Mode=_TXRX;
	USART_Handle.USART_Config.USART_NoOfStopBits=STOPBITS_1;
	USART_Handle.USART_Config.USART_ParityControl=PARITY_DISABLE;
	USART_Handle.USART_Config.USART_WordLength=USART_8BITS;
	USART_Init(&USART_Handle);
}

//////////////////////////////////////////////////COM4 CONFIGURATION////////////////////////////////////////////////////////////////////////////////

void COM4_GPIO()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_10;//TX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=8;
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_11;//RX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=8;
	GPIO_Init(&GPIO_Btn);
}

void COM4()
{
	COM4_GPIO();
	USART_Handle_t USART_Handle;
	USART_Handle.pUSARTx=UART4;
	USART_Handle.USART_Config.USART_Baud=BAUD_19200;
	USART_Handle.USART_Config.USART_Mode=_TXRX;
	USART_Handle.USART_Config.USART_NoOfStopBits=STOPBITS_1;
	USART_Handle.USART_Config.USART_ParityControl=PARITY_DISABLE;
	USART_Handle.USART_Config.USART_WordLength=USART_8BITS;
	USART_Init(&USART_Handle);
}


//////////////////////////////////////////////////PS4 Configuration function/////////////////////////////////////////////////////////////

//////////////////////////////////////////////////PS4 UART PINS CONFIGURATION/////////////////////////////////////////////////////////// 
void uart_GPIO()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=0;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_6;                 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_6;//TX
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=8;
//	GPIO_Init(&GPIO_Btn);
//	GPIO_Btn.pGPIOx=GPIOB;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_7;//RX
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_MED;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=7;
	GPIO_Init(&GPIO_Btn);
	
	

	
}
///////////////////////////////////////////PS4 UART6 CONFIGURATION///////////////////////////////////////////////////////////////////////
void ps2_uart_config()
{
	uart_GPIO();
	USART_Handle_t USART_Handle;
	USART_Handle.pUSARTx=USART1;
	USART_Handle.USART_Config.USART_Baud=BAUD_115200;
	USART_Handle.USART_Config.USART_Mode=_RX;
	USART_Handle.USART_Config.USART_NoOfStopBits=STOPBITS_1;
	USART_Handle.USART_Config.USART_ParityControl=PARITY_DISABLE;
	USART_Handle.USART_Config.USART_WordLength=USART_8BITS;
	USART_Init(&USART_Handle);
}

//////////////////////////////////////////ALL UART CONFIGURATION END/////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// ////IMU GPIO CONFIGURATION//////////////////////////////////////////////////
void Spi_GPIO()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_5;                  //CLOCK OF SPI
	GPIO_Init(&GPIO_Btn);
	//GPIO_Btn.pGPIOx=GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
  GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	//GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PUUP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_6;                  //MISO OF SPI
	GPIO_Init(&GPIO_Btn);
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;       
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_7;                  //MOSI OF SPI
	GPIO_Init(&GPIO_Btn);
	//GPIO_Btn.pGPIOx=GPIOA;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_4;                  //SS PIN AS GPIO
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
  //GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
	GPIO_Write(GPIOA,GPIO_4,SET);
}

////////////////////////////////////////////////////IMU SPI CONFIGURATION//////////////////////////////////////////////////
void IMU()
{
	Spi_GPIO();
	SPI_Handle_t SPI_Handle;
	SPI_ClockControl(SPI2,ENABLE);
  SPI_Handle.pSPI=SPI2;
	
	SPI_Handle.SPIConfig.CPHA=CPHA_0;
	SPI_Handle.SPIConfig.CPOL=CPOL_0;
	SPI_Handle.SPIConfig.SSM=SSM_DI;
	SPI_Handle.SPIConfig.FF=MSB;
  SPI_Handle.SPIConfig.DeviceMode=SPI_MASTER;
	SPI_Handle.SPIConfig.BusConfig=Half_Duplex;
	SPI_Handle.SPIConfig.DFF=SPI_8_BIT;
	SPI_Handle.SPIConfig.Frequency=SPI_F_32;
  SPI_Init(&SPI_Handle);
	
	IMU_TIMER=&get_rate; /////////////////////////PASSING FUNCTION POINTER/////////////////////////////
 	IMU_TIMER_CONF();    /////////////////////////TIMER  FUNCTION////////////////////////////////////// 
	

}
//////////////////////////////////////////    ///IMU TIMER INTERRUPT FUNCTION//////////////////////////////////////////////
void get_rate()
{	
	 
	
  GPIO_Write(GPIOB,GPIO_12,RESET);
	res[0]=SPI_Communicate(SPI2,0x80);
  res[1]=SPI_Communicate(SPI2,0x00);
	res[2]=SPI_Communicate(SPI2,0x00);
	res[3]=SPI_Communicate(SPI2,0x00);
  GPIO_Write(GPIOB,GPIO_12,SET);
	
	if(res[0]!=0xFF)
	{
		imu_con=1;
	}
	else
	{
		imu_con=0;
	}
	
	   ans=(res[3]|(res[2]<<8)|(res[1]<<16)|(res[0]<<24));
		count = (ans & (0x07<<29))>>29;	  
	//count = 1;//(ans & 0xe0000000)>>29;
	   ans=((ans & 0x001FFFE0)>>5);
	
	      if(ans>32767)
				{ ans=ans-(65536);}                               
	
	
	ans1 = (double)ans-87.0;//-92.2;//96//81.4;																																														
	
	if((ans1<=20)&&(ans1>=-20))
	{
		ans1=(ans1)/800;
	}
	else
	{
		ans1=(ans1)/80;
	}

	  ang+=(ans1*0.002553);//02);//0.0022222);
	pos();
	SPEED();
	
//	
//	if(readings)
//	{	
//		
//			i++;
//		j=i/20;
//		if(j<100)
//		{
//			array_x[j]=x_dis;
//			array_y[j]=y_dis;
//		}
//		
//	}
	
	
	
	return;

}
///////////////////////////////////////////////////IMU TIMER CONFIGURATION/////////////////////////////////////////////////
void IMU_TIMER_CONF()
{
	TimerPeriClock(TIM6,ENABLE);
	Timer_Handle_t Timer;
	Timer.pTIMx=TIM6;
	Timer.Timer_Config.prescaler=0x53;
	Timer.Timer_Config.period=0x9C3;//0x8AD;//0x9C3;
	Timer.Timer_Config.Mode=0;
	Timer_Init(&Timer);
	Timer_IRQconfig(TIM6_DAC_IRQn,ENABLE);
}

/////////////////////////////////////////////////////IMU CONFIGURATION END//////////////////////////////////////////////////
//////////////////////////////////////////////////DRIVING PARTS//////////////////////////////////////////////////////////////////////////////
void pos()
{
	x_d_dis=0-x_*Encoder_Con;
	y_d_dis=0-y_*Encoder_Con;
	
	th=(ang*Degree_to_Rad);
	
	x_dis+=((x_d_dis-prev_x_d)*cos(th))-((y_d_dis-prev_y_d)*sin(th));
	y_dis+=((y_d_dis-prev_y_d)*cos(th))+((x_d_dis-prev_x_d)*sin(th));
	
	prev_x_d=x_d_dis;
	prev_y_d=y_d_dis;
	
	return;

}
void encoder_vel()
{
	if(countcycle>0)
	{	
		x_vel=(x_dis-prev_x)/(0.1*countcycle);
		y_vel=(y_dis-prev_y)/(0.1*countcycle);
		countcycle=0;
		prev_x=x_dis;
	  prev_y=y_dis;

		ang_vel=atan2(y_vel,x_vel);
		r_vel=hypot(x_vel,y_vel);
		
	}
	
	return;
}
void General_purpose_timer()
{
	te=&SPEED;//assigning the address of function to null pointer
	TimerPeriClock(TIM7,ENABLE);
	Timer_Handle_t Timer;
	Timer.pTIMx=TIM7;
	Timer.Timer_Config.prescaler=0x68F;
	Timer.Timer_Config.period=0x31;//0x8AD;//0x9C3;
	Timer.Timer_Config.Mode=0;
	Timer_Init(&Timer);
	Timer_IRQconfig(TIM7_IRQn,ENABLE);
}
void timer()
{
	countcycle++;
	//countcycle1++;
	
}
void SPEED()
{
	//pos();
	//counter++;	
	s_tick=x_dis;
	x_vel=(s_tick-s_p_tick)*4;///0.1;
	
	if(fabs(x_vel)>9)
		Speedx=0.8*x_vel;
	else
		Speedx=1.4*x_vel;
	s_p_tick=s_tick;
	
	s_y_tick=y_dis;
	y_vel=(s_y_tick-s_y_p_tick)*4;///0.1;

	if(fabs(y_vel)>9)
		Speedy=0.8*y_vel;
	else
		Speedy=1.4*y_vel;
	
	s_y_p_tick=s_y_tick;
	
		ang_vel=atan2(y_vel,x_vel);
		r_vel=hypot(Speedx,Speedy);
}



//////////////////////////////////////////////////////PLUS LINE SENSOR /////////////////////////////////////////////////////////////

void line_sense()
{
	/*
																							6=	2=	U3					
																							7=	1= U2
																							2=	0= U1
												4= 5=L3	5= 4=L2	8= 3=L1					0= 6=R1	3= 7=R2	1= 8=R3
	
	i = ranges from (0-8)
	*/
	for(int i=0;i<9;i++)	
	{ 
		if(i==2)          // CHANNEL - 0                 
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==7)			// CHANNEL - 3
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==6)    // CHANNEL - 12
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
			mux_delay;
		}

		else if(i==8)    // CHANNEL - 4
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		else if(i==5)  // CHANNEL - 6
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		else if(i==4)
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}	
		else if(i==0)
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_SET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		else if(i==3)
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		else if(i==1)
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_SET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		h_plus_sensor[i]=read_sensor;
	}


//	cls();
//lcd(h_plus_sensor[5]);
//lcd(h_plus_sensor[4]);
//lcd(h_plus_sensor[3]);
//lcd(h_plus_sensor[6]);
//lcd(h_plus_sensor[7]);
//lcd(h_plus_sensor[8]);
//lowerline();
//	
//lcd(h_plus_sensor[2]);
//lcd(h_plus_sensor[1]);
//lcd(h_plus_sensor[0]);	
	
	net_error_v_plus=0;
	net_error_h_plus=0;
	sum_sensor_v=0;
	sum_sensor_h=0;		
	if(h_plus_sensor[0]==1)
	{
		net_error_v_plus-=1;	
		sum_sensor_v+=1;		
	}
	if(h_plus_sensor[1]==1)
	{
		net_error_v_plus-=2;
		sum_sensor_v+=1;	
	}
	if(h_plus_sensor[2]==1)
	{
		net_error_v_plus-=3;
		sum_sensor_v+=1;	
	}
	
	
	
	if(h_plus_sensor[6]==1)
	{
		net_error_h_plus-=1;
		sum_sensor_h+=1;	
	}
	if(h_plus_sensor[7]==1)
	{
		net_error_h_plus-=2;
		sum_sensor_h+=1;	
	}
	if(h_plus_sensor[8]==1)
	{
		net_error_h_plus-=3;
		sum_sensor_h+=1;	
	}
	
	if(h_plus_sensor[3]==1)
	{
		net_error_h_plus+=1;
		sum_sensor_h+=1;	
	}
	if(h_plus_sensor[4]==1)
	{
		net_error_h_plus+=2;
		sum_sensor_h+=1;	
	}
	if(h_plus_sensor[5]==1)
	{
		net_error_h_plus+=3;
		sum_sensor_h+=1;	
	}

}


void plus_line_detect()
{     
//===================================================================================================
/*
DESCRIPTION : This function is used for taking values from new plus sensor patti 

1.channels of mux are used to switch each sensor one by one  	
2.each sensor output is stored in array 
3.from that array, error is calculated
4.The channels of mux are written below with respective sensor[0,3,12,4,7,5,6,9,10,11,2,1,8]

THE CONVENTION FOF PLUS LINE SENSOR
*/
  /*   
       _               								@ 9   	
	    | |              								@ 10    
      | |            	                @ 11 
	    | --     @       @       @      @       @       @       @
	    | __    0(i=0)  3(i=1)  12(i=2) 4(i=3)  7(i=4)  5(i=5)    6(i=6)
	    | |              								@ 2 
      |_|      								        @ 1 
      FRC        								      @ 8
 

*/
  //===================================================================================================	

///////////////////////////////////////   FOR HORIZONTAL ROW (ONE WITH FRC) 	
for(int i=0;i<7;i++)	
{ 
		if(i==0)          // CHANNEL - 0                 
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==1)			// CHANNEL - 3
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
			C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==2)    // CHANNEL - 12
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_SET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
			mux_delay;
		}

		else if(i==3)    // CHANNEL - 4
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==4)   // CHANNEL - 7
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==5)   // CHANNEL - 5 
		{
			A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
			B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}

		else if(i==6)  // CHANNEL - 6
		{
			A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
			B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
			C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
			D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
			mux_delay;
		}
		h_plus_sensor[i]=read_sensor;
}

///////////  used in place of sensor_val_update_h 
	sensor_value[0] = h_plus_sensor[0];
  sensor_value[1] = h_plus_sensor[1];
	sensor_value[2] = h_plus_sensor[2];
	sensor_value[3] = h_plus_sensor[3];
	sensor_value[4] = h_plus_sensor[4];
	sensor_value[5] = h_plus_sensor[5];
	sensor_value[6] = h_plus_sensor[6];

	sum_sensor_h = 0;

for(int i=0;i<7;i++)                                               ///  [for(int i=2;i<6;i++)] // FOR USE WITH OLD SENSOR PATTI 
	{
		sum_sensor_h += sensor_value[i];	
	}

///////////      FOR VERTICAL SENSING  PATTI 
for(int i=0;i<7;i++)       	
{
if(i==0)            
{
	A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
	B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
	C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_SET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
	mux_delay;
}

else if(i==1)
{
	A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
	B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
	C_RESET; //GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_SET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
	mux_delay;
}

else if(i==2)
{
	A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
	B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
	C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_SET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
	mux_delay;
}

else if(i==3)
{
	A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
	B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
	C_SET;//GPIO_Write(GPIOD,GPIO_4,SET);//MUXC
	D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
	mux_delay;
}

else if(i==4)
{
	A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
	B_SET;//GPIO_Write(GPIOD,GPIO_3,SET);//MUXB
	C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
	mux_delay;
}

else if(i==5)
{
	A_SET;//GPIO_Write(GPIOD,GPIO_1,SET);//MUXA
	B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
	C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_RESET;//GPIO_Write(GPIOD,GPIO_2,RESET);//MUXD
	mux_delay;
}

else if(i==6)
{
	A_RESET;//GPIO_Write(GPIOD,GPIO_1,RESET);//MUXA
	B_RESET;//GPIO_Write(GPIOD,GPIO_3,RESET);//MUXB
	C_RESET;//GPIO_Write(GPIOD,GPIO_4,RESET);//MUXC
	D_SET;//GPIO_Write(GPIOD,GPIO_2,SET);//MUXD
	mux_delay;
}
v_plus_sensor[i]=read_sensor;
}

///////////  used in place of sensor_val_update_v

	static double net_err=0;	
	/*
  	lcd(Pin0_19); //5
		lcd(Pin0_20); //n
		lcd(Pin0_21); //3
		lcd(Pin0_22); //6
		lcd(Pin2_4);  //1
		lcd(Pin2_7);  //4
	*/
	
	sensor_value[0] = v_plus_sensor[0];
  sensor_value[1] = v_plus_sensor[1];
	sensor_value[2] = v_plus_sensor[2];
	sensor_value[3] = v_plus_sensor[3];
	sensor_value[4] = v_plus_sensor[4];
	sensor_value[5] = v_plus_sensor[5];
	
	sum_sensor_v = 0;

for(int i=2;i<6;i++)
	{
		sum_sensor_v += sensor_value[i];	
	}	

////////////////////////////////////  ERROR CALCULATION FOR HORIZONTAL STRIP 
if(h_plus_sensor[0])
error_h_plus[0]=4;      // 5

if(h_plus_sensor[1])
error_h_plus[1]=2;      // 4 

if(h_plus_sensor[2])
error_h_plus[2]=1;      // 3 

if(h_plus_sensor[3])
error_h_plus[3]=0;      // 0

if(h_plus_sensor[4])
error_h_plus[4]=(-1);   // -3

if(h_plus_sensor[5])
error_h_plus[5]=(-2);   // -4

if(h_plus_sensor[6])
error_h_plus[6]=(-4);   // -5

//avoid repetition

if(h_plus_sensor[0] && h_plus_sensor[1])
error_h_plus[0]=2;      // 4 

if(h_plus_sensor[1] && h_plus_sensor[2])   
error_h_plus[1]=1;      // 3 

//avoid repetition
if(h_plus_sensor[6] && h_plus_sensor[5])
error_h_plus[6]=(-2);   //-4

if(h_plus_sensor[5] && h_plus_sensor[4])   
error_h_plus[5]=(-1);   //-3 
//
////////////////////////////////////  ERROR CALCULATION  FOR VERTICAL STRIP 
if(v_plus_sensor[0])
error_v_plus[0]=4;      // 5

if(v_plus_sensor[1])
error_v_plus[1]=2;      // 4 

if(v_plus_sensor[2])
error_v_plus[2]=1;      // 3

if(v_plus_sensor[3])
error_v_plus[3]=0;      //0

if(v_plus_sensor[4])
error_v_plus[4]=(-1);   // -3

if(v_plus_sensor[5])
error_v_plus[5]=(-2);   // -4

if(v_plus_sensor[6])
error_v_plus[5]=(-4);   // -5

//avoid repetition

if(v_plus_sensor[0] && v_plus_sensor[1])
error_v_plus[0]=2;

if(v_plus_sensor[1] && v_plus_sensor[2])   
error_v_plus[1]=1;

//avoid repetition
if(v_plus_sensor[6] && v_plus_sensor[5])
error_v_plus[6]=(-2);

if(v_plus_sensor[5] && v_plus_sensor[4])   
error_v_plus[5]=(-1);


net_error_h_plus = error_h_plus[0]+error_h_plus[1]+error_h_plus[2]+error_h_plus[3]+error_h_plus[4]+error_h_plus[5]+error_h_plus[6];

net_error_v_plus = error_v_plus[0]+error_v_plus[1]+error_v_plus[2]+error_v_plus[3]+error_v_plus[4]+error_v_plus[5]+error_v_plus[6];


error_h_plus[0]=0;
error_h_plus[1]=0;
error_h_plus[2]=0;
error_h_plus[3]=0;
error_h_plus[4]=0;
error_h_plus[5]=0;
error_h_plus[6]=0;

error_v_plus[0]=0;
error_v_plus[1]=0;
error_v_plus[2]=0;
error_v_plus[3]=0;
error_v_plus[4]=0;
error_v_plus[5]=0;
error_v_plus[6]=0;
/*
////////////////////////////////   RIGHT JUNCTION COUNT

if(h_plus_sensor[3] && h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4] &&  !(h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2]) )
{
right_junction_flag=1;
}

if(right_junction_flag && !(h_plus_sensor[3] && h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4] && !(h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2]) ))	
  {
  right_junction++;
  right_junction_flag=0;
  }
////////////////////////////////   LEFT JUNCTION COUNT

else if(h_plus_sensor[3] && h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2] && !(h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4]))
{
left_junction_flag=1;
}

if(left_junction_flag && !(h_plus_sensor[3] && h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2] && !(h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4])))	
  {
  left_junction++;
  left_junction_flag=0;
  }

////////////////////////////////// FULL JUNCTION COUNT
	
else if(h_plus_sensor[3] && h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2] && h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4])
{
full_junction_flag=1;
}

if(full_junction_flag &&!(h_plus_sensor[3] && h_plus_sensor[0] && h_plus_sensor[1] && h_plus_sensor[2] && h_plus_sensor[6] && h_plus_sensor[5] && h_plus_sensor[4]))
  {
  full_junction++;
  full_junction_flag=0;
  }
//return;
*/


//..............   debugging snippet
/*
//set(muxA);
//set(muxB);

//cls();
//lcd(Pin2_4);
//lcd(Pin2_5);
//set(Port2_5);
//set(Port0_8);
//lowerline();

//lcd(h_sensor[]);

cls();
lcd(h_plus_sensor[0]);
lcd(h_plus_sensor[1]);
lcd(h_plus_sensor[2]);
lcd(h_plus_sensor[3]);
lcd(h_plus_sensor[4]);
lcd(h_plus_sensor[5]);
lcd(h_plus_sensor[6]); 

lowerline();	
*/
cls();
lcd(h_plus_sensor[0]);
lcd(h_plus_sensor[1]);
lcd(h_plus_sensor[2]);
lcd(h_plus_sensor[3]);
lcd(h_plus_sensor[4]);
lcd(h_plus_sensor[5]);
lcd(h_plus_sensor[6]); 
lowerline();
lcd(v_plus_sensor[0]);
lcd(v_plus_sensor[1]);
lcd(v_plus_sensor[2]);
lcd(v_plus_sensor[3]);
lcd(v_plus_sensor[4]);
lcd(v_plus_sensor[5]);
lcd(v_plus_sensor[6]); 

}
//______________________________________________________________________________________________________________________

void control_pid_3W_omni(double dr_angle,double upperlimit)
{
	/*__________________________CONTROL_PID_OMNI()___________________________
	Uses/Method:      when provided with direction of motion and the maximum 
                    speed it drives the omni wheel drive while staying in limits
                    set by lim11, lim12, lim21, lim22 and the value of icontrol
                    and keeps the value given to motor in check.
	Returns :         void
	Scope:            universal                                          */
	
	/*
	Parameters:-	dr_angle is the angle the robot has to follow;
	              upperlimit  is the max speed with which the motor can move.

	Use:- used to drive omni robot which holds its orientation using pid which utilises the value of kp,ki & kd;

	Notes:-
	-lim11,lim12,lim21,lim22 are limits which control speed of motor. these limits are slowly changed to accelerate the bot slowly.
	 change is made in drive() function.
	-if constant axis is to be maintained(i.e the bot will move towards a given point without effect of orientation), then
	 instead of passing angle which is to be followed as the parameter, pass (angle_to_be_followed - current_angle_of_robot)
	 as parameter.
	-icontrol is the parameter which limits the PID control value. If set too high then drastic oscillations will be produced.
	 if set too low then the robot can't maintain constant angle.
	-value_1, value_2, value_3 can be calculated and derived using simple trigonometric functions.
	-wheels are numbered along with their respective variables in the order shown below:
	-if angle greater than 2PI is sent, it'll hold its angle without moving(this is for user's convenience, that is if the user
	 want's robot to stop but still wants application of PID
	                                              3
	                                             ___
	                                           /     \
	                                          /       \
	                                         /         \
	                                         \         /
																			   2  \__ __ _/  1

	                                         ( operator )

																				   COM1- motor 3
																		 COM0 channel 1 - motor 2
																		 COM0 channel 2 - motor 1
	*/
			int Ay_f=0,Ax_f=0;    

	
	if(imu_con==0)///*count!=2)             //this is controlled in Gyroscope interrupt named get_rate(). This checks the permanently fixed bits.
	 {
		 lowerline();
		 lcd((char*)"Gyro removed!!");
		 while(imu_con==0)
		 {
			 USART_SendData(USART3,63);
			 USART_SendData(USART2,63);
			 USART_SendData(USART3,191);
			 USART_SendData(USART2,191);
		 }
		 lowerline();
		 lcd((char*)"                ");
	 }
	 
//				if (upperlimit>=64)
//					upperlimit=63;
//				if(upperlimit<-63)
//					upperlimit=-63;
//		
		
			
	difference=hold_angle-ang;
				
	if(abs(dr_angle)<(10*PI))      //Driving in a particular direction not asked to hold angle
	{
		
		Ax_f = upperlimit*cos(dr_angle) ;//+((x_Vel-prev_x_Vel)*2.1));
		Ay_f = upperlimit*sin(dr_angle) ;//+((y_Vel-prev_y_Vel)*2.1);
	
	  value_1 = Ax_f*0.866 - Ay_f*0.5;
	  value_2 = Ax_f*0.866 + Ay_f*0.5;
	  value_3 = Ay_f;
	}
	else //Hold angle command
	{
		value_1=0;
		value_2=0;
		value_3=0;
		dr_angle=0;
	}
	
		
//convention of motor changed later.
	value_1=0+value_1;
	value_2=0+value_2;
	value_3=0-value_3;
	

	
//-------------------------PID Algorithm-----------------------------------
	if(difference!= 0)	
	{
			
		//-----------------Proportional------------------------
		proportional = difference * kp;
		//------------------Integrative-------------------------	
		integral += difference;
		integrald = integral * ki;
		//------------------Derivative-------------------------
		rate = difference-prevposition;
		prevposition = difference;             //Store error for use with Kd term (i.e. Rate of change of error)
		derivative = rate * kd;
		//--------------------Control--------------------------
		control = proportional+derivative+integrald;
		integral /=(1.3);	
		//--------------------PID Ends-------------------------

		//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
		if(control>icontrol)
		{	
			control=icontrol;
		}
		else if(control<(0-icontrol))
		{		
			control=(0-icontrol);
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		control=0-control;     // Just inversion of values. Was getting inverted output. :p

		value_1 -= control;
		value_2 += control;
		value_3 += control;
		
	}
	
	 
	
	int max_value=0;
	if(fabsl(value_1) >= fabsl(value_2))
	{
		if(fabsl(value_1) >= fabsl(value_3))
		{
			(max_value)=(value_1);
		}
		else
		{
			(max_value)=(value_3);
		}
	}
	else if(fabsl(value_2) >= fabsl(value_3))
	{
		  max_value=(value_2) ;
	}
	else
	{
		max_value=(value_2) ;
	}

	if(fabsl(max_value)>=64)
	{
		float multiplier=63/fabsl(max_value);
		
		value_1 *= multiplier;
		value_2 *= multiplier;
		value_3 *= multiplier;
		//lcd("//");
	}
		//Still check if limits will not hamper the motion of other motors
		//___ Send the Values____


	  USART_SendData(USART2,191 + (signed int)value_1); 
    USART_SendData(USART2,64 + (signed int)value_2);	
		USART_SendData(USART3,64 + (signed int)value_3);

	return;
}

void control_pid_omni_with_speed(double dr_angle,double upperlimit)
{
    
	  double diff_x=0,diff_y=0;
		int Ay_f=0,Ax_f=0;  
	dr_angle-=(ang*Degree_to_Rad);
  
	//	dr_angle=dr_angle-(ang*Degree_to_Rad);
	
	if(imu_con==0)///*count!=2)             //this is controlled in Gyroscope interrupt named get_rate(). This checks the permanently fixed bits.
	 {
		 lowerline();
		 lcd((char*)"Gyro removed!!");
		 while(imu_con==0)
		 {
			 USART_SendData(USART3,63);
			 USART_SendData(USART2,63);
			 USART_SendData(USART3,191);
			 USART_SendData(USART2,191);
		 }
		 lowerline();
		 lcd((char*)"                ");
	 }
	 
				if (upperlimit>=64)
					upperlimit=63;
				if(upperlimit<-63)
					upperlimit=-63;
		
		
			
	difference=hold_angle-ang;
				
	if(abs(dr_angle)<(10*PI))      //Driving in a particular direction not asked to hold angle
	{
		
		diff_x = upperlimit*cos(dr_angle)-Speedx;
		diff_y = upperlimit*sin(dr_angle)-Speedy;
		
		diff_x =diff_x*0.9;
		diff_y =diff_y*0.9;
		
		if(diff_x>=20)
			diff_x=20;
		
		if(diff_y>=20)
			diff_y=20;
		
		if(diff_x<=-63)
			diff_x=-63;
		
		if(diff_y<=-63)
			diff_y=-63;
		
		Ax_f = upperlimit*cos(dr_angle) + diff_x;//+((x_Vel-prev_x_Vel)*2.1));
		Ay_f = upperlimit*sin(dr_angle) + diff_y;//+((y_Vel-prev_y_Vel)*2.1);
	
	/*	lcd(Ax_f);
		lowerline();
		lcd(Ay_f);*/
	   value_1=(-Ax_f+Ay_f)*0.707;
	  value_2=-(Ax_f+Ay_f)*0.707;
	  value_3=(Ax_f-Ay_f)*0.707;		
		value_4=(Ax_f+Ay_f)*0.707;
	}
	else //Hold angle command
	{
		value_1=0;
		value_2=0;
		value_3=0;
		value_4=0;
		dr_angle=0;
	}
	
	
		
	difference=hold_angle-ang;
	//-------------------------PID Algorithm-----------------------------------
		if(difference!= 0)	
		{
		//-----------------Proportional------------------------
		proportional = difference * kp;
		//------------------Integrative-------------------------	
		integral += difference*(0.025);
		integrald = integral * ki;
		//------------------Derivative-------------------------
		rate = difference-prevposition;
		prevposition = difference;             //Store error for use with Kd term (i.e. Rate of change of error)
		derivative = rate * kd;
		//--------------------Control--------------------------
		control = proportional+derivative+integrald;
		integral /=(1.3);	
		//--------------------PID Ends-------------------------
		//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
		if(control>icontrol)
		{	
			control=icontrol;
		}
		else if(control<(0-icontrol))
		{		
			control=(0-icontrol);
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		control=0+control;     // Just inversion of values. Was getting inverted output. :p
		
		value_1 += control;
		value_2 += control;
		value_3 += control;
		value_4 += control;
	}
		
	double max_val;
	max_val=(value_1>value_2)?(value_1>value_3)?(value_1>value_4)?value_1:value_4:(value_3>value_4)?value_3:value_4:(value_2>value_3)?(value_2>value_4)?value_2:value_4:(value_3>value_4)?value_3:value_4;

	
	if(fabs(max_val)>63)
	{
		double multiplier=63/fabs(max_val);
		tx_1 = value_1*multiplier;
		tx_2 = value_2*multiplier;
		tx_3 = value_3*multiplier;
		tx_4 = value_4*multiplier;
	}
	else
	{
		tx_1=value_1;
		tx_2=value_2;
		tx_3=value_3;
		tx_4=value_4;
		
	}
	
				USART_SendData(USART2,63+tx_1);
				USART_SendData(USART2,190+tx_2);
				USART_SendData(USART3,63+tx_3);
				USART_SendData(USART3,191+tx_4);
	
}



void control_pid_omni(double dr_angle,double upper_speed)
{
	/*
																			 ^ (+ve)x-axis
																			 |
																	_____|_____
															 1 /		 |	 	 \ 4
																/			 |		  \
																|			 |		  |
			(+ve)y-axis  <------------|-------		  |
									  						|						  |
																\						  /
															 2 \___________/ 3
													
																	( operator )
	
	com2 : ch1-Motor 1 ; ch2-Motor 2 
	com3 : ch1-Motor 3 ; ch2-Motor 4
	
	*/	
	 // double diff_x=0,diff_y=0;

		double Ay_f=0,Ax_f=0;    
	
	 if(count!=2)             //this is controlled in Gyroscope interrupt named get_rate(). This checks the permanently fixed bits.
	 {
		 lowerline();
		 
		 lcd((char*)" Gyro removed!!");
		 while(count!=2)
		 {
			// USART_SendData(UART4,192);
			 USART_SendData(USART3,63);
			 USART_SendData(USART2,63);
			 USART_SendData(USART3,191);
			 USART_SendData(USART2,191);
		 }
		 lowerline();
		 lcd((char*)"                ");
	 }
	if(abs(dr_angle)<(10*PI))      //Driving in a particular direction not asked to hold angle
	{
		Ax_f = upper_speed*cos(dr_angle);// 2times- Speedx;
		Ay_f = upper_speed*sin(dr_angle);// 2times- Speedy;
		
	  value_1=(-Ax_f+Ay_f)*0.707;
	  value_2=-(Ax_f+Ay_f)*0.707;
	  value_3=(Ax_f-Ay_f)*0.707;		
		value_4=(Ax_f+Ay_f)*0.707;
	}
	else //Hold angle command
	{
		value_1=0;
		value_2=0;
		value_3=0;
		value_4=0;
		dr_angle=0;
	}
	
	
		
	difference=hold_angle-ang;
	//-------------------------PID Algorithm-----------------------------------
		if(difference!= 0)	
		{
		//-----------------Proportional------------------------
		proportional = difference * kp;
		//------------------Integrative-------------------------	
		integral += difference*(0.025);
		integrald = integral * ki;
		//------------------Derivative-------------------------
		rate = difference-prevposition;
		prevposition = difference;             //Store error for use with Kd term (i.e. Rate of change of error)
		derivative = rate * kd;
		//--------------------Control--------------------------
		control = proportional+derivative+integrald;
		integral /=(1.3);	
		//--------------------PID Ends-------------------------
		//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
		if(control>icontrol)
		{	
			control=icontrol;
		}
		else if(control<(0-icontrol))
		{		
			control=(0-icontrol);
		}
		///////////////////////////////////////////////////////////////////////////////////////////////////////////

		control=0+control;     // Just inversion of values. Was getting inverted output. :p
		
		value_1 += control;
		value_2 += control;
		value_3 += control;
		value_4 += control;
	}
		
	double max_val;
	max_val=(value_1>value_2)?(value_1>value_3)?(value_1>value_4)?value_1:value_4:(value_3>value_4)?value_3:value_4:(value_2>value_3)?(value_2>value_4)?value_2:value_4:(value_3>value_4)?value_3:value_4;

	
	if(fabs(max_val)>63)
	{
		double multiplier=63/fabs(max_val);
		tx_1 = value_1*multiplier;
		tx_2 = value_2*multiplier;
		tx_3 = value_3*multiplier;
		tx_4 = value_4*multiplier;
	}
	else
	{
		tx_1=value_1;
		tx_2=value_2;
		tx_3=value_3;
		tx_4=value_4;
		
	}
	
				USART_SendData(USART2,63+tx_1);
				USART_SendData(USART2,191+tx_2);
				USART_SendData(USART3,63+tx_3);
				USART_SendData(USART3,191+tx_4);
	
	
}
void motor_pid(int16_t syn_ang,int16_t buffer)
{

	//		GetQEICount();
		cur_tick=s_;
			syn_tick	=	syncro_chain_constant*(syn_ang);//-ang);//(-ang) added on 25 august
			dif_tick=syn_tick-cur_tick;

			if((dif_tick>buffer)||(dif_tick<(0-buffer)))
			{
//				while((dif_tick>buffer)||(dif_tick<(0-buffer)))
//				{
					//==============proportional===================
					proprnl=s_p*dif_tick;
					//==============differential===================
					difrnl=s_d*(prev_error-dif_ang);
					prev_error=dif_tick;
					//==============integral=======================
					integrl+=dif_tick;
					integrld=s_i*integrl;
					integrl /=1.2;
					
					//---------------PID Algorithm-------------------
					contrl=proprnl+difrnl+integrld;
					if(contrl>icontrl)
					{contrl=icontrl;}
					else if(contrl<(0-icontrl))
					{contrl=0-icontrl;}
					
					//control=-control;//to invert the effect
					if(contrl < 0)
					{
						USART_SendData(UART4,192-6+contrl);
					}
					else if (contrl > 0)
					{
						USART_SendData(UART4,192+6+contrl);
					}
					v=1;
//				}
			}
			else
			{
				v=0;
				USART_SendData(UART4,192);
			}
		
		//	waitms(1);
	return;
}

void control_syncro_pid(int16_t syn_ang,uint16_t hold_speed,uint8_t Motion)
{
/*
	Safety precautions before using this function!!
	
1.	In UART mode, (control + speed) must not be greater than 64.
2.	Not to set control too high otherwise bot will Hay-Wire.
3.	Not to set it too low as it won't be able to correct the orientation angle of bot
4.	angle of motion is controlled by syn _ang
5.	

	4 wheel synchronous drive convention to be followed
											^
											|
	x=+ve ; y=+ve				|			x=+ve ; y=-ve
								 _____|_____
								|	.		|		.	|
		fl					|	.		|		.	|		fr
________________|_____|_____|____________________
								|	.		|		.	|
								|	.		|		.	|
								|_____|_____|
											|
	x=-ve ; y=+ve				|			x=-ve ; y=-ve
											|
		bl								|					br
											|									 
*/
	if(count!=2)             //this is controlled in Gyroscope interrupt named get_rate(). This checks the permanently fixed bits.
	{
		lowerline();
		lcd((char*)" Gyro removed!!");
		while(count!=2)
		{
			USART_SendData(UART4,192);
			 USART_SendData(USART3,64);
			 USART_SendData(USART1,64);
			 USART_SendData(USART3,191);
			 USART_SendData(USART1,191);
		}
		lowerline();
		lcd((char*)"                ");
	}
	
	//for hold velocity ==speed
		pos();
	
		vel_error=hold_speed-(r_vel/1.62);//equation to changed according to bot
		if(vel_error!=0)
		{
				//-----------------Proportional------------------------
				proportional = vel_error * kpv;
				//-------------------Integral--------------------------
				integral += vel_error;
				integrald = integral * kiv;
				integral /= 1.3;	
				//------------------Derivative-------------------------
				rate = prevposition - vel_error;
				prevposition = vel_error;  
				derivative = rate * kdv;
				//--------------------Control--------------------------
			speed_control = (proportional+integrald+derivative);
			
			//control limit
				if(speed_control>ispeed_control)
				{	speed_control=ispeed_control;}
				else if(speed_control<(0-ispeed_control))
				{	speed_control=0-ispeed_control;}
			speed_control=(1)*speed_control;//to invert the effect of pid just multiply by -1
				
speed= hold_speed+speed_control;
				
				//max limit for safety
				if(speed>maxi_speed)
				{	speed=maxi_speed;}
				else if(speed<0-maxi_speed)
				{	speed=-maxi_speed;	}
				
				//for negative speed 
				if(speed<0)
				{
					speed=(-1)*speed;
					if(syn_ang>0)
						syn_ang=180-syn_ang;
					else
						syn_ang=180+syn_ang;
				}
		}
		else //vel_error=0
		{
speed=hold_speed;
		}
		
//for accelaration settings change minor_speed or accelaration constant
		
//		f_speed=(2.1 )*(r_vel+3.1)/1.72;//accelaration constant=1.2
//		if(f_speed>speed)
//		{
			f_speed=speed;
//		}
		
//to differentiate whether the bot has to move forward or backward

			cur_tick=s_;
			dif_ang=syn_ang-(cur_tick/syncro_chain_constant);
			if(dif_ang>95)				
			{	
				syn_ang=syn_ang-180;
				invert=-1;					
			}
			else if(dif_ang<-95)	
			{
				syn_ang=syn_ang+180;
				invert=-1;					
			}
			else	
			{				
				invert=1;
			}
			
//giving motor angle command
//-------------------syncro motor angle------------			
			
			motor_pid((syn_ang-(invert*(ang-hold_angle))),150);//100 buffer//-ang
			
			
			
			s_ang = Degree_to_Rad*(-(cur_tick/syncro_chain_constant));//syn_ang);// angle of chain used by syncro equations


//for hold angle
			difference=hold_angle-ang;
			if((difference!=0))	
			{
				//-----------------Proportional------------------------
				proportional = difference * kp;
				//-------------------Integral--------------------------
				integral += difference;
				integrald = integral * ki;
				//------------------Derivative-------------------------
				rate = prevposition - difference;
				prevposition = difference;  
				derivative = rate * kd;
				//--------------------Control--------------------------
				control = R*(proportional+integrald+derivative);
				integral /= 1.3;
				//--------------------PID Ends-------------------------
				//limit on control parameter: so that if angle changes more than a limit, the motors don't go hay-wire.
				if(control>icontrol)
				{control=icontrol;}
				else if(control<(0-icontrol))
				{control=(0-icontrol);}
					
				control=(0 - invert*control); //Inversion of values for inverted output. :p
				
				fl = (invert)*(f_speed + control*sin(fl_offset+s_ang));
				fr = (invert)*(f_speed + control*sin(fr_offset+s_ang));
				br = (invert)*(f_speed + control*sin(br_offset+s_ang));
				bl = (invert)*(f_speed + control*sin(bl_offset+s_ang));
				//just multiply with invert or put if condition to invert
			
				USART_SendData(USART1,191-fl);
				USART_SendData(USART1,63+fr);
				USART_SendData(USART3,191+br);
				USART_SendData(USART3,63-bl);
//				}
			}
			else	//differece==0 ;)
			{
				if(invert==-1)
				{
					USART_SendData(USART1,192-2+f_speed);
					USART_SendData(USART1,64-1-f_speed);
					USART_SendData(USART3,192-1-f_speed);
					USART_SendData(USART3,64+f_speed);;
				
				}
				
				else if(invert==1)
				{
					USART_SendData(USART1,191-f_speed);
					USART_SendData(USART1,64+f_speed);
					USART_SendData(USART3,192+f_speed);
					USART_SendData(USART3,64-1-f_speed);;
				
				}
			}
			
		//important delay WHEN EXTPRINTBIN IS USED
		//	waitms(1);

			
	return;
			
}

void dead_recon(double x_final,double y_final,int req_speed,int buffer)
{
	navin=1;
	while(navin)
	{
//		pos();
//		encoder_vel();
		d_x = x_final-temp_x;
		d_y = y_final-temp_y;
		a_x=  (x_final+temp_x)/2;
		a_y = (y_final+temp_y)/2;
		diff_x = x_final-x_dis;
		diff_y = y_final-y_dis;
		dr_ang = (atan2(diff_y,diff_x));// in radian
		//dis = hypot(diff_x,diff_y);

		prevd_x=diff_x;
		prevd_y=diff_y;
		
//		if(  (diff_x>-200) && (buffer==1)  )		
//		{
//			navin=0;
//		}
//		else if ((diff_x<200) && (buffer==2) )
//		{
//			navin=0;
//		}
		if((d_x==0) && (d_y==0) && (buffer==1))
		{
			navin=0;
		}
		else if( (x_dis>a_x) && (d_x>0) && (d_y==0) && (buffer==1) )
		{
			navin=0;
		}	
		else if( (x_dis<a_x) && (d_x<0) && (d_y==0) && (buffer==1))
		{
			navin=0;
		}
		else if(  ( (y_dis-a_y) > ((-1)*(d_x/d_y)*(x_dis-a_x)) )   &&  (d_y>0) && (buffer==1) )	 //((dr_ang)<PI/2)  )		
		{
			navin=0;
		}
		else if (  ((y_dis-a_y) < ((-1)*(d_x/d_y)*(x_dis-a_x)) )   &&  (d_y<0) && (buffer==1) )	//
		{
			navin=0;
		}
		else if ((abs(diff_x)<5) && (abs(diff_y)<5) && (buffer==0))
		{
			navin=0;
			goto next;
		}
		else if ((abs(diff_x)<20) && (abs(diff_y)<20) && (buffer==2))
		{
			navin=0;
			goto next;
		}
		control_pid_omni_with_speed(dr_ang,req_speed);
		next:
		;
	}
	
	temp_x=x_final;
	temp_y=y_final;
//	temp_x1=temp_x;
//	temp_y1=temp_y;
//	temp_x2=temp_x1;
//	temp_y2=temp_y1;
//	temp_x3=temp_x2;
//	temp_y3=temp_y2;


	return;
}
void Ps4_drive(int16_t angle_of_motion)//Old Function not in use
{
/*=====================================================================================================================================================
temp_four2
================================================THIS IS DONE FOR  SUDDEN CHANGE IN DIRECTION ===========================================================
*/
double temp_four2,diff_dr_ang;	
		Ps2_val_update();

//===========================================  MINOR ADJUSTMENT CHECK  ======================================================
	  if(ps_down && ps_left)
	  {			
			while(ps_down && ps_left)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();	
			  control_pid_omni(0.75*PI,minor_speed);// 61				
			}
	}	
		
		else if(ps_down && ps_right)
		{
			while(ps_down && ps_right)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();		
				control_pid_omni(-0.75*PI,minor_speed);// 61				   //(ORIGNAL) control_pid_omni(-(3*PI)/4-angle_of_motion,minor_speed);// 61	
			}
		}			

		else if(ps_up && ps_left)
		{
			while(ps_up && ps_left)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code)
			{
				Ps2_val_update();								
				control_pid_omni(0.25*PI,minor_speed);// 61				
			}
		}			
		
		else if(ps_up && ps_right)
		{
			while(ps_up && ps_right)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code)
			{
				Ps2_val_update();	
				control_pid_omni(-0.25*PI,minor_speed);// 61				
			}
		}
		else if(ps_right)
		{
			while(ps_right)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();
				control_pid_omni(-PI/2,minor_speed);// 61		 //should have been PI, but using this to reduce somr weird error		
			}
		}
		
		else if(ps_left)
		{
			while(ps_left)// && !BUTTON_ONE && !BUTTON_TWO && !BUTTON_THREE)       //BUTTON_ONE ... BUTTON_THREE is theme specific. Remove it when using it as general code
			{
				Ps2_val_update();	
				control_pid_omni(PI/2,minor_speed);				  //should have been zero, but using this to reduce somr weird error
			}
		}

		else if(ps_up)
		{			
			while(ps_up)
			{					
				Ps2_val_update();	
				control_pid_omni(0,minor_speed);           /////// ((PI/2)-angle_of_motion,minor_speed)				
			}
		}

		else if(ps_down)
		{
			while(ps_down)
			{
				Ps2_val_update();
				control_pid_omni(PI,minor_speed);	       ///(-(PI/2)-angle_of_motion,minor_speed)			  
			}
		}		
		else
		{		
			if(turn_final==1)            /// FOR    ANTI-[CLOCKWISE
			{
				if(abs(ang-hold_angle)<3)        ///keep it 3
					hold_angle += 10;//hold_angle-=ang_increment;
				turn_yes=1;                    // -25
			}							
			else if(turn_final==-1)                           /////    FOR CLOCKWISE
			{
				if(abs(ang-hold_angle)<3)        ///keep it 3
					hold_angle -= 10;//hold_angle-=ang_increment;
				turn_yes=1;                    // -25
			}
			else if(turn_yes>=1)
			{
				turn_yes++;
				if(turn_yes>4)
					turn_yes=0;
				hold_angle=ang;
			}  
			
			if(goti_drive)
			{
//				diff_dr_ang=temp_four2-temp_four;
//				temp_four2=temp_four;
//				if((fabs(diff_dr_ang)>40)&&(fabs(diff_dr_ang)<280))
//				{
//					kp=0.65;
//					kd=5;
//				}
//				else
//				{
//					kp=0.6;
//					kd=2.2;
//				}
//				cls();
//				lcd(temp_four);
				control_pid_omni(temp_four,major_speed);
			}
			else //hold_angle
			{	
//				{
					control_pid_omni(0,0);
//				}
			}		
		}
		
		return ;
}




void actuations(void)
{
		if(ps_cross)/// XXXXXXXX
		{
			
			if(flap_count==1)
			{
				Flap_Set;	
				flap_count=0;
				
			}		
			else //if(flap_count==0)//1;
			{	
				Flap_Reset;
				flap_count=1;
			}waitms(300);
			while(ps_cross)
			{
				cls();
				Ps2_val_update();
				//waitms(10);
			}
			
		}
		
		if(ps_square) //[][][][][][][]
		{
			if(kapda_count==0)
			{
				Kapda_Set;
				kapda_count=1;
			}
			else // if(kapda_count==1)
			{
				Kapda_Reset; 
				kapda_count=0;
			}waitms(300);
			while(ps_square)	
			{
				cls();
				Ps2_val_update();

			}
			
		}
//		if(ps_circle)	//OOOOOOOOOOOOOOOOO
//		{
//				set(PR_KICK);
//				count_cycle1=0;			
//		}
		if(ps_triangle) /////\/\/\/\/\/
		{
				
			if(kick_count==0)
			{
				Kick_Set;
				kick_count=1;
			}
			else //if(kick_count==1)
			{
				Kick_Reset;
				kick_count=0;
			}		waitms(300);
			while(ps_triangle)
			{
				cls();
				//
				Ps2_val_update();
				
			}
			
		}
		
		
//		if(ps_triangle)/////\/\/\/\/\/
//		{
//				set(PASS);
//				count_cycle1=0;			
//		}
		
		
}

void ps4_drive(double angle_of_motion,int16_t major_speed,int16_t minor_speed)//New Function
{
	static double drive_angle,drive_speed;
	Ps2_val_update();
	
	//for goti
	
	if(goti_drive)
	{	
//		if(temp_four!=400 && temp_four!=360)
//		{ 
			drive_angle=temp_four;
			drive_speed=major_speed;
//		}
//		else
//		{
//			drive_angle=0;
//			drive_speed=0;
//		}
	}
	else if(goti_drive==0)
	{
		if(ps_down && ps_left)
		{
			drive_angle=135*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_down && ps_right)
		{
			drive_angle=-135*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_up && ps_left)
		{
			drive_angle=45*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_up && ps_right)
		{
			drive_angle=-45*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_down)
		{
			drive_angle=180*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_left)
		{
			drive_angle=90*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_up)
		{
			drive_angle=0*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else 	if(ps_right)
		{
			drive_angle=-90*Degree_to_Rad;
			drive_speed=minor_speed;
		}
		else
		{
			drive_angle=0;
			drive_speed=0;
		}
	}
//for hold_angle
			if(turn_final==1)            /// FOR    ANTI-[CLOCKWISE
			{
				if(abs(ang-hold_angle)<3)        ///keep it 3
					hold_angle += 5;//20;				//hold_angle-=ang_increment;
				turn_yes=1;                    // -25
			}
						
			else if(turn_final==-1)                           /////    FOR CLOCKWISE
			{
        if(abs(ang-hold_angle)<3)        ///keep it 3
					hold_angle -= 5;//20;				//hold_angle-=ang_increment;
				turn_yes=1;                    // -25
			}
		
			else if(turn_yes>=1)
			{
				turn_yes++;
				if(turn_yes>4)
					turn_yes=0;
				hold_angle=ang;
			} 
		
		//	actuations();			
//			cls();
//			lcd(drive_angle,2);
//			lcd(" ");
//			lcd(drive_speed,1);
//			lowerline();
//			lcd(hold_angle,1);
			control_pid_omni(drive_angle,drive_speed);
			
			
}


void Ps2_val_update()//new
{
	
	/*
	updates the variables named after ps2 buttons, call it just as is to update all values.
	put Bluetooth UART in COM2
	QUE:- is interrupt to ATmega better for speed?
	*/
	
	ps_up=ps_right=ps_left=ps_down=ps_square=ps_triangle=ps_cross=ps_select=ps_start=ps_circle=ps_r1=ps_r2=ps_l1=ps_l2=ps_touchpad=ps_l3=0;
		
	//  int temp_four1=0;
		static int temp_four3;
		
	//___________________________________________________________________________________________________________________________
	  //----------------------------------- INPUT FROM COM2 STORED IN BYTES  ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	        PS4_GPIO_SET;              //GPIO to arduino 
	           ps2_val[0]=USART_ReceiveData(USART1);     //buttons
	           ps2_val[1]=USART_ReceiveData(USART1);		 //buttons
	           ps2_val[2]=USART_ReceiveData(USART1);		 //7-4 bits for left joy 12 angles plus 4 centre & 3-2 bits for right joy turn 
	        PS4_GPIO_RESET;
	//___________________________________________________________________________________________________________________________
	  //------------------------ IF COMMUNICATION HAS STOPPED, THEN STOP THE BOT FROM MOVING   ----------------------------------------------------------
	  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if(uart_timeout>1000)
		{
			lowerline();
			lcd((char*)"Ps4 Disconnected");
			//USART_SendData(UART4,192);
			 USART_SendData(USART3,64);
			 USART_SendData(USART2,64);
			 USART_SendData(USART3,191);
			 USART_SendData(USART2,191);
			while(uart_timeout>1000)                     // SAFETY BECOMES ZERO IN INPUTBIN() COMMAND -> SAFETY INCREASES IN TIMER-2 INTERRUPT
			{
				 PS4_GPIO_SET;              //GPIO to arduino 
				 USART_ReceiveData(USART1);     //buttons
				 PS4_GPIO_RESET;
			}
			lowerline();
			lcd((char*)"                ");
		}
//		cls();
//		lcd(ps2_val[0]);
//		lcd("|");
//		lcd(ps2_val[1]);
//		lowerline();
//		lcd(ps2_val[2]);		
		//___________________________________________________________________________________________________________________________
    //-----------------------Byte decoding section; numbering of bytes, seeking actually pressed buttons, etc ---------------------
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  int u;
	for(u=0;u<3;u++)  //u<4
    {
			temp_four3 = ps2_val[u];
			 for(int i=0;i<8;i++)
			 {
				 ip[u][i]=temp_four3%2;
				 temp_four3=temp_four3/2;
			 }
		}
																														
	if(ip[0][0]==1)																						
		{ps_triangle=1; }// lcd("Triangle");}                   
	if(ip[0][1]==1)                                           
		{ps_circle=1;   }// lcd("Circle");}                     
	if(ip[0][2]==1)                                           
		{	ps_cross=1;		 }//lcd("Cross");}							        
	if(ip[0][3]==1)                                           
		{	ps_square=1;	}//	lcd("Square");}                     
	if(ip[0][4]==1)                                           
		{	ps_up=1;		}// lcd("UP");}                           
	if(ip[0][5]==1)                                           
		{	ps_right=1;	}// lcd("RIGHT");}                      
	if(ip[0][6]==1)                                        
		{	ps_down=1;	}//	lcd("DOWN");}                        
	if(ip[0][7]==1)                                        
		{	ps_left=1;	}//	lcd("LEFT");}	                      
		                                                        
	if(ip[1][0]==1)                                           
		{ps_l2=1;     }//   lcd("L2");}                         
	if(ip[1][1]==1)                                           
		{ps_r2=1;      }//  lcd("R2");}                         
	if(ip[1][2]==1)                                           
		{ps_l1=1;       }// lcd("L1");}                         
	if(ip[1][3]==1)                                           
		{ps_r1=1;       }// lcd("R1");}                         
	if(ip[1][4]==1)                                           
		{	ps_select=1;	}//	lcd("SHARE");}//	lcd("SELECT");}   
	if(ip[1][5]==1)                                           
		{  ps_start=1;   }//  lcd("START");}                    
	if(ip[1][6]==1)                                          
		{ ps_touchpad=1; }// lcd("TOUCHPAD");}                   
	if(ip[1][7]==1)                                           
		{ ps_l3=1; }//lcd((char*)"L3");}                         
		                                                        
	//to get turn command 
	if((ip[2][2]==1)&&(ip[2][3]==0))
	{	turn_final=-1;}
	else if((ip[2][2]==0)&&(ip[2][3]==1))
	{  turn_final=1;}
	else
	{	turn_final=0;}
		
		
		
		// to get adc values for driving

	
		if((ip[2][4]==0)&&(ip[2][5]==0)&&(ip[2][6]==0)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=0;}
		else if((ip[2][4]==1)&&(ip[2][5]==0)&&(ip[2][6]==0)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=35*(PI/180);}
		else if((ip[2][4]==0)&&(ip[2][5]==1)&&(ip[2][6]==0)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=55*(PI/180);}
		else if((ip[2][4]==1)&&(ip[2][5]==1)&&(ip[2][6]==0)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=90*(PI/180);}
		else if((ip[2][4]==0)&&(ip[2][5]==0)&&(ip[2][6]==1)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=125*(PI/180);}
		else if((ip[2][4]==1)&&(ip[2][5]==0)&&(ip[2][6]==1)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=145*(PI/180);}
		else if((ip[2][4]==0)&&(ip[2][5]==1)&&(ip[2][6]==1)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=180*(PI/180);}
		else if((ip[2][4]==1)&&(ip[2][5]==1)&&(ip[2][6]==1)&&(ip[2][7]==0))
		{	goti_drive=1;
		temp_four=-35*(PI/180);}
		else if((ip[2][4]==0)&&(ip[2][5]==0)&&(ip[2][6]==0)&&(ip[2][7]==1))
		{	goti_drive=1;
		temp_four=-55*(PI/180);}
		else if((ip[2][4]==1)&&(ip[2][5]==0)&&(ip[2][6]==0)&&(ip[2][7]==1))
		{	goti_drive=1;
		temp_four=-90*(PI/180);}
		else if((ip[2][4]==0)&&(ip[2][5]==1)&&(ip[2][6]==0)&&(ip[2][7]==1))
		{	goti_drive=1;
		temp_four=-125*(PI/180);}
		else if((ip[2][4]==1)&&(ip[2][5]==1)&&(ip[2][6]==0)&&(ip[2][7]==1))
		{	goti_drive=1;
		temp_four=-145*(PI/180);}		
		
		else
		{
			goti_drive=0;
			temp_four=0;
		}
		
		
		//lcd(temp_four*(180/PI));
		return;
		
}

	








void receive_ball_1()
{
			int x,sp,lim=60;

		/////////////////////////////////////////////Section - 1///////////////////////////////////////////////////////////

		//----------------------------------------Accelaration Starts----------------------------------------------------
		for(x=0;x<=4700;x+=400)
		{
			
			sp=12+(1.2)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(0-x,0,sp);
		}
		//------------------------------------------Accelaration Ends-----------------------------------------------------------	
		//-----------------------------------------Deaccelaration starts-----------------------------------------------------
		for(x=0;x<1800;x+=400)
		{
			sp=(0.4)*(r_vel);//Deaccelarate
			if(r_vel<=15)
			{
				sp=15;
			}
			dead_recon(-(x+4700),-x/10,sp);
			if(x>300)
			{
				hold_angle-=5;
			}
		}//hold_angle=60;
		
		hold_angle=-30;
		dead_recon(-6550,-200,7);
		dead_recon(-6600,-200,5);
		//-----------------------------------------Deaccelaration ends--------------------------------------------------------
		//-----------------------------------------Maintaining-part-----------------------------------------------------------
	//	k=2000;
		ps_l1=0;
		while(ps_l1==0)
		{
			Ps2_val_update();

			if(r_vel>=1)
			{
				dead_recon(-6670,-200,4,0);
			}
			else
			{
				cls();
				lcd(x_dis);
				lowerline();
				lcd(y_dis);
				control_pid_omni_with_speed(0,0);
			}
		}
	//-----------------------------------END-OF-SECTION-1-------------------------------------------------------------------------
	
}
void try_ball_1()
{
		hold_angle=0;
		for(int i=0;i<300;i++)
			control_pid_omni(0,0);
	
	
		int x,sp,lim=35;
		//----------------------------------------Accelaration Starts----------------------------------------------------
		for(x=0;x<=2200;x+=400)
		{
			sp=12+(0.9)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6700+(0.091*x)),-x-200,sp);
		}
		for(x=0;x<=1800;x+=400)
		{
			sp=5+(1)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6900-(0.195*x)),-x-2400,sp);
		}
		
		for(x=0;x<=1600;x+=400)
		{
			sp=10+(0.4)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6550+(0.53*x)),-(x+4200),sp);
		}
			ps_l1=0;
		while(ps_l1==0)
		{
			Ps2_val_update();
			if(r_vel>=1)
			{
				dead_recon(-7400,-5800,4,0);
			}
			else
			{
				cls();
				lcd(x_dis);
				lowerline();
				lcd(y_dis);
				control_pid_omni_with_speed(0,0);
			}
		
			
/*
			int sp;
			line_sense();
			if((net_error_v_plus==0)&&(net_error_h_plus==0))
			{
				drive_ang=-3*PI/4;
				sp=6;
			}
			else if(net_error_v_plus==0)
			{
				if(net_error_h_plus>0)
				{
					drive_ang=-5*PI/12;
				}
				else
				{
					drive_ang=-7*PI/12;
				}
				sp=6;
			}
			else if(net_error_h_plus==0)
			{

					drive_ang=-PI/2;
					sp=6;
				if(sum_sensor_h>4	)
				{
					drive_ang=0;
					sp=0;
				}
				if(sum_sensor_v<=2)
				{
					drive_ang=-PI;
					sp=6;
				}
			}
			else if ((net_error_v_plus!=0)&&(net_error_h_plus!=0))
			{
				drive_ang=atan2(net_error_v_plus,net_error_h_plus);
				sp=4;
			}
			
			control_pid_omni_with_speed(drive_ang,sp);
		*/
		}


	
}


void receive_ball_2()
{
	int x,sp,lim=35;
		//----------------------------------------Accelaration Starts----------------------------------------------------
		for(x=0;x<=1400;x+=380)//1500
		{
			sp=12+(0.9)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(7400-(0.55*x)),-(5800-x),sp);
		}
		for(x=0;x<=2000;x+=400)
		{
			sp=8+(1)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6550+(0.18*x)),-(4400-x),sp);
		}
		
		for(x=0;x<=2200;x+=400)
		{
			sp=8+(0.4)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6910-(0.091*x)),-(2400-x),sp);
		}
		
		ps_l1=0;
		while(ps_l1==0)
		{
			Ps2_val_update();
			hold_angle=-30;
	
			if(r_vel>=1)
			{
				dead_recon(-6700,-200,4,0);
			}
			else
			{
				cls();
				lcd(x_dis);
				lowerline();
				lcd(y_dis);
				control_pid_omni_with_speed(0,0);
			}
		}
//	while(1)
//	{	
//		cls();
//		lcd(x_dis);
//		lowerline();
//		lcd(y_dis);
//		control_pid_omni(0,0);
//	}
}
void try_ball_2()
{
		hold_angle=0;
		for(int i=0;i<300;i++)
			control_pid_omni(0,0);
		
		int x,sp,lim=35;
		//----------------------------------------Accelaration Starts----------------------------------------------------
		for(x=0;x<=2200;x+=400)
		{
			sp=12+(0.9)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6700+(0.091*x)),-x-200,sp);
		}
		for(x=0;x<=2600;x+=400)
		{
			sp=5+(0.9)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6230-(0.257*x)),-x-2400,sp);
		}
		
		for(x=0;x<=700;x+=300)
		{
			sp=8+(0.1)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6000-(0.257*x)),-(x+5000),sp);
		}
		
		ps_l1=0;
		while(ps_l1==0)
		{
			Ps2_val_update();
			if(r_vel>=1)
			{
				dead_recon(-6000,-5700,4,0);
			}
			else
			{
				cls();
				lcd(x_dis);
				lowerline();
				lcd(y_dis);
				control_pid_omni_with_speed(0,0);
			}
		}
}
void receive_ball_3()
{
	int x,sp,lim=35;
		//----------------------------------------Accelaration Starts----------------------------------------------------
		for(x=0;x<=3500;x+=350)
		{
			sp=12+(0.9)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6000+(0.257*x)),x-5700,sp);
		}
		for(x=0;x<=1000;x+=300)
		{
			sp=4+(0.8)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon(-(6900-(0.1*x)),x-2200,sp);
		}
		
		for(x=0;x<=1000;x+=300)
		{
			sp=8+(0.2)*(r_vel);//accelaration constant=1.2
			if(sp>lim)
			{
				sp=lim;
			}
			dead_recon((-6800-(0.1*x)),x-1200,sp);
		}
		
		ps_l1=0;
		while(ps_l1==0)
		{
			hold_angle=-30;
			Ps2_val_update();
			if(r_vel>=1)
			{
				dead_recon(-6700,-200,4,0);
			}
			else
			{
				cls();
				lcd(x_dis);
				lowerline();
				lcd(y_dis);
				control_pid_omni_with_speed(0,0);
			}
		}

}
void plus_line_sensor_config()
{
	GPIO_Handle_t GPIO_Btn;
	GPIO_Btn.pGPIOx=GPIOD;                
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_UHS;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_10;	//A 
	GPIO_Init(&GPIO_Btn);
	
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_8;  //B
	GPIO_Init(&GPIO_Btn);
	
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_9;	//SENSOR INPUT
	GPIO_Init(&GPIO_Btn);   	
	
	GPIO_Btn.pGPIOx=GPIOE; 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_UHS;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_13;	//C 
	GPIO_Init(&GPIO_Btn);
	
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_15;	//D
	GPIO_Init(&GPIO_Btn);	
	
}

void servo_confi()
{
	GPIO_Handle_t GPIO_Btn;
  GPIO_Btn.pGPIOx=GPIOD;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinAltFunMode=2;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_12;                  
	GPIO_Init(&GPIO_Btn);
	
	
	TimerPeriClock(TIM4,ENABLE);
	TIM4->PSC=0x54;
	TIM4->CCMR1|=(0<<0);//O/P FIRST CANNEL
	TIM4->CCER|=(1<<1);//Active Low
	TIM4->CCMR1|=(7<<4);
	TIM4->ARR=0x4E1F;
	
	TIM4->CCMR1|=(1<<3);
	TIM4->CR1|=(1<<7);
	TIM4->CCER|=(1<<0);
	TIM4->CR1|=(1<<0);
	
}
void relay_board()
{
	GPIO_Handle_t GPIO_Btn;
	GPIO_Btn.pGPIOx=GPIOE;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_3;                 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
	
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_6;                 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
	
	
	GPIO_Btn.pGPIOx=GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_13;                 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
	
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_9;                 
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinOPtype=GPIO_OUT_PP;
//	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
//	GPIO_Init(&GPIO_Btn);
	
	
	
}
void laser_config()
{
	GPIO_Handle_t GPIO_Btn;
	GPIO_Btn.pGPIOx=GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber=GPIO_9;                 
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_LOW;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NPULL;
	GPIO_Init(&GPIO_Btn);
	
	
}
/*
*--------------------------------------------NVIC INTERRUPT CONFIGURATION-------------------------------------------------------------------
*--------------All interrpts will be configured in below section and pointer will be passed to desired function-----------------------------
*/
extern "C"
{
 void TIM6_DAC_IRQHandler()
	{
		TIM6->SR&=(~(1<<0));//do not delete
		(*IMU_TIMER)();
	}
	
	void TIM7_IRQHandler()
	{
		TIM7->SR&=~(1<<0);
		//GPIOB->ODR^=(1<<12);
		(*te)();
	}
}
