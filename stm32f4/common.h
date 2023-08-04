#include <stm32f407xx.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx_1.h"
#include "GPIO.h"
#include "SPI.h"
#include "I2C.h"
#include "USART.h"
#include "RCC.h"
#include "TIMER.h"
#include "LCD.h"

#ifndef _common_H
#define _common_H

//////////////////////////////////////////////ENCODER DEFINATIONS///////////////////////////////////////////////////////////
#define x_  (signed)(TIM2->CNT)
#define y_  (signed)(TIM5->CNT)
#define temp_s  (int16_t)(TIM3->CNT)
#define s_  (0-temp_s)
//#define x_dis x_*Encoder_Con
//#define y_dis y_*Encoder_Con

#define Encoder_Con 0.088970867
#define syncro_chain_constant 22.78
#define x_t -x_*Encoder_Con
#define y_t -y_*Encoder_Con
#define PI 3.1415926535
#define Degree_to_Rad 0.0174532925
#define Rad_To_Degree 57.29577951308232087679
#define DIA	56.25
#define CNTPERREV 2048.0
#define Degree_to_Rad 0.0174532925
#define P1 0.01002
#define P2 1.111

#define Discontinous 0
#define Continous 1
#define maxi_speed 60
#define R 33
#define fl_offset 0.828			//47.436			//1
#define fr_offset -0.828		//-47.436			//2
#define br_offset 3.9695		//180+47.436			//3
#define bl_offset 2.3137		//180-47.436			//4
#define b_const 477.46483 //75/(9*Degree_to_Rad)


#define Kick_Set GPIO_Write(GPIOE,GPIO_3,SET)
#define Kick_Reset GPIO_Write(GPIOE,GPIO_3,RESET)

#define Flap_Set GPIO_Write(GPIOE,GPIO_6,SET)
#define Flap_Reset GPIO_Write(GPIOE,GPIO_6,RESET)

#define Kapda_Set GPIO_Write(GPIOC,GPIO_13,SET)
#define Kapda_Reset GPIO_Write(GPIOC,GPIO_13,RESET)

#define PS4_GPIO_SET GPIO_Write(GPIOB,GPIO_6,SET)
#define PS4_GPIO_RESET GPIO_Write(GPIOB,GPIO_6,RESET)

#define A_SET GPIO_Write(GPIOD,GPIO_10,SET)
#define B_SET GPIO_Write(GPIOD,GPIO_8,SET)
#define C_SET GPIO_Write(GPIOE,GPIO_13,SET)
#define D_SET GPIO_Write(GPIOE,GPIO_15,SET)
#define A_RESET GPIO_Write(GPIOD,GPIO_10,RESET)
#define B_RESET	GPIO_Write(GPIOD,GPIO_8,RESET)
#define C_RESET GPIO_Write(GPIOE,GPIO_13,RESET)
#define D_RESET GPIO_Write(GPIOE,GPIO_15,RESET)

#define read_sensor GPIO_Read(GPIOD,GPIO_9) 

#define mux_delay waitus(100)
#define servo_value(x) TIM4->CCR1=x
/*
*-----------------------------------------Variables Declaration Part------------------------------------------------------
*/
////////////////////////////////////////////////PS4 VARIABLES////////////////////////////////////////////////////////////////
extern bool ip[4][8],ps_up,ps_right,ps_down,ps_left,ps_square,ps_triangle,ps_circle,ps_cross,ps_select,ps_start,
	    ps_r1,ps_r2,ps_l1,ps_l2,ps_touchpad,ps_l3,goti_drive;
extern int ps2_val[4], one,two,three,four,turn_adc,turn_yes;
extern uint8_t left_joy,right_joy;
extern double temp_four;
extern int turn_final,major_speed,minor_speed;
extern bool kick_count,kapda_count,flap_count;
extern int ps2_minor_speed,ps2_major_speed,countcycle,countcycle1;
///////////////////////////////////////////////IMU VARIABLES/////////////////////////////////////////////////////////////////
extern double ang,ans1;
extern double count;
extern bool imu_con;

//line sensor - line sensor PID
extern double proportional_line_sense, kp_line_sense, integral_line_sense, integrald_line_sense, ki_line_sense, rate_line_sense, prev_err, derivative_line_sense, kd_line_sense, 
	     control_line_sense, icontrol_line_sense;
extern int sum_sensor_v,sum_sensor_h;
extern bool sensor_value[8];
extern bool h_plus_sensor[16],v_plus_sensor[16];           /////  FOR PLUS SENSOR
extern double  error_h_plus[7],error_v_plus[7],net_error_v_plus,net_error_h_plus,left_junction,right_junction,right_junction_flag,left_junction_flag,full_junction_flag, full_junction;
extern double new_line_heading,new_kp_line; 
extern		double drive_ang;
extern int x_temp,y_temp;
///////////////////////////////////////////////DRIVE VARIABLES////////////////////////////////////////////////////////////////
extern double lim11_lim,lim12_lim,lim21_lim,lim22_lim;
extern double lim11,lim12;             //limits for channel 1 -> with stop value 64    80,48
extern double lim21,lim22;           //limits for channel 2 -> with stop value 192   208,176
extern int lim,control_speed,flag,previous,prevposition;
extern double acc;
extern int base_value;
extern char dummyl,dummyr,txt1,txt2,txt3;
extern float kp,ki,kd;
extern double b_heading,hold_angle,value_1,value_2,value_3,value_4, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol,difference;
extern double Speed;
extern int counter,prv_counter,tick_diff,tx_1,tx_2,tx_3,tx_4;
extern double data[50];
//PID variables
extern double hold_angle,value_1,value_2,value_3, proportional, integral,derivative,integrald,rate,
	     control,old_control,icontrol;
extern float kp,ki,kd,manual_kp,manual_ki,manual_kd,auto_kp,auto_ki,auto_kd;
extern char dummyl,dummyr,txt1,txt2,txt3;
//
//velocity feedback
extern double Speedx,s_tick,s_p_tick,s_y_tick,s_y_p_tick,Speedy;
extern double x_dis,y_dis,x_d_dis,y_d_dis;
extern int16_t array_x[100],array_y[100],readings;
extern int i,j;
extern double x_vel,y_vel,r_vel,ang_vel,prev_x,prev_y;
extern int temp_x,temp_y,temp_x1,temp_x2,temp_x3,temp_y1,temp_y2,temp_y3;
//DED recon - encoder
extern int b1,a1,previous,tickl,tickr,prvvaluel,prvvaluer,prevposition,templ ,tempr ;
extern double th;
extern double left_ddis,dis_per_count_left,right_ddis,dis_per_count_right,left_dia,right_dia;
//
extern float Vx,Vy,kpv,kdv,kiv,prevd_x,prevd_y,integ_x,integ_y;

///syncro drive
extern double	diff_x,diff_y,dis,dr_ang,ang_d, coor_spd,base_spd;
extern bool v;
	extern int fl,fr,bl,br;
	extern double cur_tick,s_ang, dif_ang,dif_tick,prev_error, integrld,temp_tick, syn_tick,icontrl,contrl,proprnl,difrnl,integrl;
	extern double vel_error,speed_control,ispeed_control;
	extern	double s_p,s_d,s_i;
	extern	signed int  rot_spd;
	extern	int8_t navin;

////////////////////////////////////////////////DEAD RACON VARIABLES////////////////////////////////////////////////////////
extern double d_kp,d_kd,d_ki,d_error,ideal,dr_error;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void jtag();
void uart_GPIO();
void ENCODER_GPIO_X();
void ENCODER();
void ENCODER_GPIO_Y();
void ENCODER_GPIO_S();
void Spi_GPIO();
void IMU();
void GPIO_Button();
void get_rate();
void Ps2_val_update();
void ps2_uart_config();
void ps2_drive(double drive_speed);
void IMU_TIMER_CONF();
//void COM0_GPIO();
void COM0();
void COM0_GPIO();
void COM1();
void COM2_GPIO();
void COM2();
void COM3_GPIO();
void COM3();
void COM4_GPIO();
void COM4();	
void timer();
void control_pid_3W_omni(double,double);
void control_syncro_pid(int16_t syn_ang,uint16_t hold_speed,uint8_t Motion=0);
void dead_recon(double x_final,double y_final,int req_speed,int buffer=1);
void motor_pid(int16_t syn_ang,int16_t buffer);
void pos(void); 
void encoder_vel(void);
void Ps4_drive(int16_t);
void ps4_drive(double angle_of_motion,int16_t major_speed,int16_t minor_speed);
void control_pid_omni_with_speed(double dr_angle,double upperlimit);
void line_sense();
void control_pid_omni(double,double);
void SPEED();
void General_purpose_timer();
void dead_racon(double x,double y,double dr_speed);\
void relay_board();
void plus_line_sensor_config();
void plus_line_detect();

void servo_confi();
void actuations(void);
void receive_ball_1();
void try_ball_1();
void try_ball_2();
void receive_ball_2();
void receive_ball_3();
void laser_config();

#endif
