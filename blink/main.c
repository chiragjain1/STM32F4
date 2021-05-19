/*
AUTHOR:CHIRAG
"""""""""""""""""""""""""""
GPIO_A IS set to output[MODER] and ODR is used for setting pins to high
The while loop is infamous blink code with two variables 
800000 is frequency  of MCU

"""""""""""""""""""""""""""
*/


#include <__cross_studio_io.h>
#include <stm32f407xx.h>
int i = 0,j=0;


void main(void)
{

  RCC->AHB1ENR |= (1<<0); //GPIO_A ENABLE
  GPIOA->MODER |= 0X00005555;//GPIOA SET AS OUPUT
  GPIOA->ODR = 0X0000FFFF;//all output is set to one initially 
  while(1)
  {
    /*while(i!=800000)
    {
      GPIOA->ODR = 0x00;
      i = i+1;
    }
    while(j!=800000)
    {
      GPIOA->ODR = 0x0000FFFF;
      j = j+1;
    }*/
      GPIOA->ODR = 0x0000FFFF;
    i=0;j=0;
  }
}
