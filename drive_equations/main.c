#include <__cross_studio_io.h>
#include <stm32f407xx.h>
#include <math.h>
#include "stm32f4xx_roboclaw.h"

#define PS_L1 0
#define PS_R1 1
#define PS_L2 2
#define PS_R2 3
#define PS_L3 4
#define PS_R3 5
#define PS_TRIANGLE 6
#define PS_SQUARE 7
#define PS_CROSS 8
#define PS_CIRCLE 9
#define PS_UP 10
#define PS_LEFT 11
#define PS_DOWN 12
#define PS_RIGHT 13
#define PS_START 14
#define PS_SELECT 15

int xj1=0,yj1=0,xj2=0,yj2=0,pot1=0,pot2=0,pot3=0,pot4=0,pwm_range=100;							//analog values(serially received from remote);
int butt[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};									//digital values(serially received from remote);

float speed,angle,squared;
double A_x,A_y,ang_s=0,error;
double k_p_lidar = 2.1 , k_i_lidar = 0 ,k_d_lidar = 0;
double F1,F2,F3,V1,V2,V3;
uint8_t RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int RX_range=200,RX_raw=255,RX_ad=255,RX_count=0;
uint8_t TX[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
int flag[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t TX_raw=200,TX_ad=201,TX_flag=0;

void usart1rx_init(void);
void receive(void);
int map_value(long,long,long,long,long);
void drive_equation(double ,double ,double );

void main (void)
{
  RCC->AHB1ENR |= (1<<3);//PORT D
  GPIOD->MODER = 0X55555555;
  RCC->APB2ENR |= (1<<14);
  usart1rx_init();
  while(1)
  {
    receive();
    drive_equation(-yj1,-xj1,yj2);
  }
}


void drive_equation(double A_x,double A_y,double ang_s)
{
    F1 = (0.5773*A_x) + (0.333*A_y) + (0.333*ang_s);
    F2 = -(0.5773*A_x) + (0.333*A_y) + (0.333*ang_s);
    F3 = (0*A_x) - (0.666*A_y) + (0.333*ang_s); 
        
    driveM1(129,F1);//F1
    driveM2(128,-F2);//F2
    driveM1(128,F3);//F3
}


void usart1rx_init()
{
  RCC->APB2ENR |= (1<<4);
  RCC->AHB1ENR |= (1<<0);
  GPIOA->MODER |= (1<<21);
  GPIOA->AFR[1] |= (1<<10) | (1<<9) | (1<<8);
  USART1->CR1 |= (1<<13) | (1<<5) | (1<<15);//enable usart and interupt
  USART1->CR2 &= ~(1<<13) & ~(1<<12);//stop 1
  USART1->GTPR |=(1<<0);//prescalar
  USART1->BRR |= (1<<11) | (1<<10) | (1<<8) | (1<<2) | (1<<1);
  USART1->CR1 |=(1<<2);//enable receive
  NVIC_EnableIRQ(USART1_IRQn);
}
void receive()
{
	yj1=map_value(RX[0],0,RX_range,(-pwm_range),pwm_range);
	xj1=map_value(RX[1],0,RX_range,pwm_range,(-pwm_range));
	yj2=map_value(RX[2],0,RX_range,(-pwm_range),pwm_range);
	xj2=map_value(RX[3],0,RX_range,pwm_range,(-pwm_range));
	
	if (butt[PS_START]==1)
	{
         // driveM1(128,0);
		butt[PS_START]=0;
	}
	if (butt[PS_SELECT]==1)
	{
		butt[PS_SELECT]=0;
	}
	if (butt[PS_UP]==1)
	{
           // driveM1(128,127);
		butt[PS_UP]=0;
	}
	if (butt[PS_DOWN]==1)
	{
         //driveM1(128,-127);
		butt[PS_DOWN]=0;
	}
	if (butt[PS_LEFT]==1)
	{
		butt[PS_LEFT]=0;
	}
	if (butt[PS_RIGHT]==1)
	{
		butt[PS_RIGHT]=0;
	}
	if (butt[PS_SQUARE]==1)
	{
		butt[PS_SQUARE]=0;
	}
	if (butt[PS_CIRCLE]==1)
	{
		butt[PS_CIRCLE]=0;
	}
	if (butt[PS_TRIANGLE]==1)
	{
		butt[PS_TRIANGLE]=0;
	}
	if (butt[PS_CROSS]==1)
	{
		butt[PS_CROSS]=0;
	}
	if (butt[PS_L1]==1)
	{
		butt[PS_L1]=0;
	}
	if (butt[PS_R1]==1)
	{
		butt[PS_R1]=0;
	}
	if (butt[PS_L2]==1)
	{
		butt[PS_L2]=0;
	}
	if (butt[PS_R2]==1)
	{
		butt[PS_R2]=0;
	}
	if (butt[PS_L3]==1)
	{
		butt[PS_L3]=0;
	}
	if (butt[PS_R3]==1)
	{
		butt[PS_R3]=0;
	}
}
int map_value(long val, long min1 , long max1, long min2, long max2)
{
	return (val - min1) * (max2 - min2) / (max1 - min1) + min2;
}


void USART1_IRQHandler()
{
  USART1->SR &= ~(1<<5);//receive
  RX_count=1;
	RX_raw=USART1->DR;
	if ((RX_raw>200) && (RX_raw<255))//201 to 216 for addresses of analog values, 231 to 246 for buttons;
	{
		RX_ad=RX_raw;
//		if ((RX_raw>210) && (RX_raw<227))
//		{
//			uint8_t r_temp0=(RX_raw-211);
//			butt_rel[r_temp0]=1;
//		}

		if ((RX_raw>230) && (RX_raw<247))
		{
			uint8_t r_temp0=(RX_raw-231);
			butt[r_temp0]=1;
		}
	}
	else if ((RX_raw>=0) && (RX_raw<201))
	{
		uint8_t r_temp1=(RX_ad-201);
		if (r_temp1<16)
		{
			RX[r_temp1]=RX_raw;
		}
	}
}

