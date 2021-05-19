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


int xj1=0,yj1=0,xj2=0,yj2=0,pot1=0,pot2=0,pot3=0,pot4=0,pwm_range=60;							//analog values(serially received from remote);
int butt[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};									//digital values(serially received from remote);
uint8_t RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int RX_range=200,RX_raw=255,RX_ad=255,RX_count=0;
uint8_t TX[16]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
int flag[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t TX_raw=200,TX_ad=201,TX_flag=0;

//ps functions
void usart1rx_init(void);
void receive(void);
int map_value(long,long,long,long,long);


//drive function
void drive_equation(double ,double ,double );
void drive_equation_rtheta(double , double);
//drive variables
double F1,F2,F3,V1,V2,V3;

//encoder 1 interface(c6,c7)
void port_c_timer_init(void);
void timer_4_encoder_init(void);
int encoder1_read=0;
int en1_val=0,en1_val1=0,en1_val2=0,en1_counter=0,en1_limit=0;
int encoder1_value();

//encoder 2 interface(e9,e11)
void port_e_timer_init(void);
void timer_1_encoder_init(void);
int encoder2_read=0;
int en2_val=0,en2_val1=0,en2_val2=0,en2_counter=0,en2_limit=0;
int encoder2_value();

//lidar (usart 4 and 6) functions:
void uart4rx_init(void);//C11 receive
void usart5rx_init(void); // pin D2 receive

//lidar variables(PORTC->all variables end with 1 and PORTD->ALL VARIABLES END WITH 2)
int uart1[9] = {0} , count1 = 0, lidar_received1 = 0,HEADER_count1 = 0,distance1 = 0;
const int HEADER=0x59; //frame header of data packageS
int uart2[9] = {0} ,count2 = 0,lidar_received2 = 0,HEADER_count2 = 0,distance2 = 0,i=0;

//gyro functions
void gyro_pid();
void usart3_init();
void USART3_receive();
void improv_gyro();
void gyropack();
//gyro variables
double funct_val,intrpt_val,start_pt,angle,improv, error1,error1_prev;
float low_lim,up_lim,error;
//gyro pid VARIABLES
double k_d_gyro = 10.17,k_p_gyro = 5 ,k_i_gyro = 0.0001;
double correct_gyro;
double rotation_value = 0;

//pll function
int clock_confg();

void all_init();

int encoder1_distance,encoder2_distance,total_distance,true_distance;
double angle_factor,rspeed;
void main (void)

  {
      RCC->APB2ENR |= (1<<14);//system configuration clock for motor driver
      clock_confg();//pll

      all_init();

      while(1)
      {
          //receive();
          gyropack(); //calling all gyro functions
          encoder1_read = encoder1_value();
          encoder2_read = encoder2_value();
          
          encoder1_distance = fabs(encoder1_read*40/17);
          encoder2_distance = fabs(encoder2_read*40/17);

          total_distance = encoder1_distance + encoder2_distance;
          true_distance = encoder1_distance+encoder2_distance;
          
          angle_factor = total_distance/10;
          if(true_distance>950)
          {
            rspeed=0;
          }
          else
          {
            rspeed=100;
          }
          drive_equation_rtheta(rspeed,angle_factor);
      }
  }

void drive_equation_rtheta(double r,double theta)
{
    double x,y;
    theta = theta*M_PI/180;
    x = r*cos(theta);
    y = r*sin(theta);
    drive_equation(-y,-x,correct_gyro);
}
void drive_equation(double A_x,double A_y,double ang_s)
{
    F1 = (0.5773*A_x) + (0.333*A_y) + (0.333*ang_s);
    F2 = -(0.5773*A_x) + (0.333*A_y) + (0.333*ang_s);
    F3 = (0*A_x) - (0.666*A_y) + (0.333*ang_s); 
        
    driveM1(129,-F3);//F1
    driveM2(128,-F2);//F2
    driveM1(128,-F1);//F3//zigbee ps
}

void port_c_timer_init()
{
    RCC->APB2ENR |= (1<<1);
    RCC->AHB1ENR |= (1<<2);
    GPIOC->MODER |= (1<<15) | (1<<13);
    GPIOC->OSPEEDR |= (1<<15) | (1<<14) | (1<<13) | (1<<12);
    GPIOC->PUPDR |= (1<<14) | (1<<12);
    GPIOC->AFR[0] |= (1<<29) | (1<<28) | (1<<25) | (1<<24);
    TIM8->PSC =120;
    TIM8->ARR =65535;
    TIM8->SMCR |= (1<<1);
    TIM8->CCMR1 |= (1<<1);
    TIM8->CCMR1 |= (1<<9);
    TIM8->CR1 |=(1<<0);
}

void port_e_timer_init()
{
  RCC->AHB1ENR |= (1<<4);//GPIOE
  GPIOE->MODER |= (1<<19)|(1<<23);//ALTERNATE FUNCTION
  GPIOE->AFR[1] |= (1<<4)|(1<<12);//ALTERNATE FUNCTION FOR HIGHER ITS 1 AND FOR LOWER ITS 0
  GPIOE->OSPEEDR |= (1<<18)|(1<<19)|(1<<22)|(1<<23);
  GPIOE->PUPDR |= (1<<18)|(1<<22);
}

int encoder1_value()
{
  en1_val = TIM8->CNT;
     if(( en1_val1 - en1_val)>32700)
      {
        en1_counter = en1_counter + 1;
      }
     if((en1_val - en1_val2)>32700)
      {
         en1_counter = en1_counter - 1;
       }
    en1_val1 = TIM8->CNT;
    en1_val2 = TIM8->CNT;
    en1_limit = en1_counter*65535;
    return en1_val+en1_limit;
}

void timer_1_encoder_init()
{
  RCC->APB2ENR |= (1<<0);//TIM1ENABLE
  TIM1->CR1 |= (1<<0);//COUNTER ENABLE
  TIM1->SMCR |= (1<<1);//ENCODER 3 MODE
  TIM1->CCMR1 |= (1<<1);//MAPPING
  TIM1->CCMR1 |= (1<<9);//MAPPING
 // TIM1->CCMR2 |= (1<<0);
 //TIM1->CCER  = 0X00;
   TIM1->PSC = 120;
  TIM1->ARR = 65535;
  TIM1->CNT = 0;
}

int encoder2_value()
{
   en2_val = TIM1->CNT;
     if(( en2_val1 - en2_val)>32700)
      {
        en2_counter = en2_counter + 1;
      }
     if((en2_val - en2_val2)>32700)
      {
         en2_counter = en2_counter - 1;
       }
    en2_val1 = TIM1->CNT;
    en2_val2 = TIM1->CNT;
    en2_limit = en2_counter*65535;
    return en2_val+en2_limit;
}

void uart4rx_init()
{
    RCC->APB1ENR |= (1<<19);
    RCC->AHB1ENR |= (1<<2);
    GPIOC->MODER |= (1<<23) | (1<<21);
    GPIOC->AFR[1] |= (1<<15) | (1<<11);
    GPIOC->OSPEEDR |= (1<<23) | (1<<22);
    GPIOC->PUPDR |=(1<<22);
    UART4->CR1 |= (1<<13) | (1<<5) | (1<<15);//enable usart and interupt
    UART4->CR2 &= ~(1<<13) & ~(1<<12);//stop 1
    UART4->GTPR |=(1<<0);//prescalar
    UART4->BRR = 0x2d5;                                 // baud-rate=115200 for 16 MHz clock
    UART4->CR1 |=(1<<2);//enable receive
    NVIC_EnableIRQ(UART4_IRQn);
}

void UART4_IRQHandler()
{
  
    i=7;
   UART4->SR &= ~(1<<5);//receive

    lidar_received1 = UART4->DR;

    if((lidar_received1 != HEADER) && (HEADER_count1 == 1))
    {
        HEADER_count1 = 0;
    }

    if((lidar_received1 == HEADER) && (HEADER_count1 == 1))
    {
      uart1[1] = HEADER;
      count1=2;
      HEADER_count1 = 2;
    }

    if((lidar_received1 == HEADER) && (HEADER_count1 == 0))
    {
      uart1[0] = HEADER;
      count1=1;
      HEADER_count1 = 1;
      i++;
    }

    if(HEADER_count1 == 2)
    {
      uart1[count1] = lidar_received1;
      count1++;
      if(count1 > 8)
      {
          count1 = 0;
          HEADER_count1 = 0;
      }

    }
}

void usart5rx_init()
{
    RCC->APB1ENR |= (1<<20);  //enable usart5
    RCC->AHB1ENR |= (1<<3);   //enable port D
    GPIOD->MODER |= (1<<5);        // alternate function on pin d2
    GPIOD->AFR[0] |= (1<<11);    // AF8 for USART5 pin d2
    UART5->CR1 |= (1<<13) | (1<<5) | (1<<15);//enable usart and interupt
    UART5->CR2 &= ~(1<<13) & ~(1<<12);//stop 1
    UART5->GTPR |=(1<<0);//prescalar
    UART5->BRR = 0x2d5;                                 // baud-rate=115200 for 16 MHz clock
    UART5->CR1 |=(1<<2);//enable receive
    NVIC_EnableIRQ(UART5_IRQn);
}

void UART5_IRQHandler()
{
  
    i=7;
    UART5->SR &= ~(1<<5);//receive

    lidar_received2 = UART5->DR;

   if((lidar_received2 != HEADER) && (HEADER_count2 == 1))
    {
      HEADER_count2 = 0;
    }

    if((lidar_received2 == HEADER) && (HEADER_count2 == 1))
    {
      uart2[1] = HEADER;
      count2=2;
      HEADER_count2 = 2;
    }

    if((lidar_received2 == HEADER) && (HEADER_count2 == 0))
    {
      uart2[0] = HEADER;
      count2=1;
      HEADER_count2 = 1;
      i++;
    }
 

    if(HEADER_count2 == 2)
    {
      uart2[count2] = lidar_received2;
      count2++;
      if(count2 > 8)
      {
        count2 = 0;
        HEADER_count2 = 0;
      }

    }
}


//GYRO FUNCTIONS
void usart3_init()
{
    RCC->APB1ENR |= (1<<18);     //USART2
    RCC->AHB1ENR |= (1<<1);     //GPIO B
    GPIOB->MODER |= (1<<23) | (0<<22) | (1<<21) | (0<<20);    //Alternate function on 10,11
    GPIOB->AFR[1] |= (1<<12) | (1<<13) | (1<<14) | (1<<10) | (1<<9) | (1<<8);
    GPIOB->OSPEEDR |= (1<<23) | (1<<22);
    GPIOB->PUPDR |=(1<<22);
    USART3->CR1 |= (1<<13); //(Usart Enable)
    USART3->CR2 &= ~(1<<13) & ~(1<<12); // 1 Stop bit
    USART3->BRR = 0X1117;//9600 pll
    USART3->CR1 |=(1<<2)|(1<<5); //Receive,Receive Interrupt
}

void USART3_receive()
{
    int gyro;
    while(!(USART3->SR & (1<<5)));
    USART3->SR &= ~(1<<5);//rxne
    USART3->SR &= ~(1<<6);//tc
    gyro= USART3->DR;
    funct_val = map_value(gyro,0,255,-127,127);
}

void improv_gyro()
{
    angle = funct_val - start_pt;
    low_lim = -127 - start_pt;
    up_lim = 127 - start_pt;
    improv = map_value(angle,low_lim,up_lim,-127,127);
    error = improv;
    error1 = error - start_pt;
    if(error1 < -127)
    {
      error1 = 255 + error1;
    }
    else if(error1 > 127)
    {
      error1 = error1 - 255;
    }
}  

void gyro_pid()
{
    double P,I,D;
 
    P = k_p_gyro*error1;
    I = k_i_gyro*error1 + I;
    D = k_d_gyro*(error1-error1_prev);
  
    if(abs(error1)<2)I=0;
    error1_prev = error1;

    correct_gyro = P + D + I;

    if(correct_gyro > 370)correct_gyro = 370;
    if(correct_gyro < -370)correct_gyro = -370;
}

void gyropack()
{
    USART3_receive();
    improv_gyro();
    gyro_pid();
}

//pll
int clock_confg()
{
 
  RCC->CR|=(1<<16);     //HSE oscillator ON
  while(!(RCC->CR & (1<<17)));  //wait till HSE oscillator ready

  FLASH->ACR |= (5<<0) | (7<<8); //Introduce a delay of 5 wait states in the flash memory to compensate for the high frequency

  RCC->CR &= ~(1<<24);    // PLL OFF
  RCC->CR|=(1<<19);    // Clock security system ON
 
  RCC->PLLCFGR &= (~63<<0);     // Clear M bits
  RCC->PLLCFGR |= (4<<0);       // M=4
  RCC->PLLCFGR &= ~(3<<16);     // Clear P bits; P =2
  RCC->PLLCFGR &= ~(1023<<6);   // Clear N bits
  RCC->PLLCFGR |= (168<<6);     // xN=168
  RCC->PLLCFGR &= ~(15<<24);    // Clear Q bits
  RCC->PLLCFGR |= (7 << 24);    // Q=7 USB OTG FS clock frequency = 48MHz
  RCC->PLLCFGR |= (1 << 22);    // Clock source for PLL is HSE

  RCC->CFGR &= ~(15<<4);
  RCC->CFGR |= (1<<12) | (1<<10); //APB1 Prescalar = 4 to get max freq 42MHz
  RCC->CFGR |= (1<<15);           //APB2 Prescalar = 2 to get max freq 84MHz
 
  RCC->CR|=(1<<24);   // PLL ON
  while(!(RCC->CR & (1<<25)));   // wait till Main PLL (PLL) clock ready flag

  RCC->CFGR &= ~(15<<0);
  RCC->CFGR |= (1 << 1);  //PLL selected as system clock
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
  USART1->BRR |=  0X445C;
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
		butt[PS_START]=0;
	}
	if (butt[PS_SELECT]==1)
	{
		butt[PS_SELECT]=0;
	}
	if (butt[PS_UP]==1)
	{
           
		butt[PS_UP]=0;
	}
	if (butt[PS_DOWN]==1)
	{
         
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

void all_init()
{
      usart1rx_init();//ps



      port_c_timer_init();//encoder 1
      port_e_timer_init();//encoder 2
      timer_1_encoder_init();

      usart3_init();//gyro
      USART3_receive();//gyro initial value
      start_pt = funct_val;

}