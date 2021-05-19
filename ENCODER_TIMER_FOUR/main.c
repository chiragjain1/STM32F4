
/*
AUTHOR:CHIRAG
"""""""""""""""""""""""""""""""""""""""""""""""""""""""
In this code timer 4[channel 1 and channel 2] has been used for using incremental encoder
GPIOD pin 12 and 13 has timer 4[ch_1 and ch_2],so we have to initialize gpiod as well and connect the two channel of encoder to gpiod_12 and
gpiod_13.As usual,AFR was used[refer to 273 of manual]

TIMER:
[REFER TO PAGE 553/554 OF STM32F4 MANUAL for understanding encoder interface]
The three important registers for resolution/taking values of encoder are ARR[AUTO-RELOAD REG],PSC[PRESCALER],CNT[COUNT]
The value of encoder is available in CNT,the value given to ARR determines the highest the encoder can count before it resets to zero.

FOR USING IN OTHER CODES:
COPY PASTE THE TWO "INIT" FUNCTIONS AND COPY THE LINE IN WHILE  
"""""""""""""""""""""""""""""""""""""""""""""""""""""""
*/

#include <__cross_studio_io.h>
#include <stm32f407xx.h>
double count=0,i=0,j=0;

void port_d_timer_init(void);
void timer_4_encoder_init(void);

int main(void)
{

  port_d_timer_init();
  timer_4_encoder_init();

  while(1)
  {
      count = TIM4->CNT;
  }
} 

void port_d_timer_init()
{
  RCC->AHB1ENR |= (1<<3);//GPIOD
  GPIOD->MODER |= (1<<25)|(1<<27);//ALTERNATE FUNCTION
  GPIOD->AFR[1] |= (1<<17)|(1<<21);//ALTERNATE FUNCTION FOR HIGHER ITS 1 AND FOR LOWER ITS 0
}

void timer_4_encoder_init()
{
  RCC->APB1ENR |= (1<<2);//TIM4ENABLE
  TIM4->CR1 |= (1<<0);//COUNTER ENABLE
  TIM4->SMCR |= (1<<1)|(1<<0);//ENCODER 3 MODE
  TIM4->CCMR1 |= (1<<0);//MAPPING
  TIM4->CCMR1 |= (1<<8);//MAPPING
 // TIM1->CCMR2 |= (1<<0);
 //TIM1->CCER  = 0X00;
  TIM4->ARR =  1024;
  TIM4->PSC = 3;
  TIM4->CNT = 0;
}