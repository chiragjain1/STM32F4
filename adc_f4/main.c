
#include <__cross_studio_io.h>
#include <stm32f407xx.h>
#include <stdio.h>
int value=10,p = 0,k=0;
void main(void)
{
     RCC->APB2ENR |= (1<<8) | (1<<14);     //ADC enabled/reset
    RCC->AHB1ENR |= (1<<0)|(1<<1);     //Enables PORTA
    GPIOA->MODER |= (1<<0) | (1<<1);   //Set Pin-A0 as to Analog Mode

    ADC->CCR |= (1<<16) | (1<<17);      //Prescalar-CLK/8
    ADC1->CR1 |= (1<<24);       //Set ADC to 8-bits  (down)
    ADC1->CR1 &= (~(1<<25));    //Set ADC to 8-bits
    ADC1->CR2 |= (1<<1) | (1<<10);    //ADC ON , Continuous conversion mode , Enables EOC bit set when ADC completes

    ADC1->SQR1 = 0;
    ADC1->SQR3 = 0;

    ADC1->CR2 |= (1<<0);
    ADC1->SMPR1|=(1<<0)|(1<<1); 

  while(1)
   {
ADC1->CR2 |= (1<<30);   //Start Conversion
    while(!(ADC1->SR & (1<<1)));    //Wait till conversion stops
    value = ADC1->DR;     //Save value
    ADC1->SR &= (~(1<<1));      //Clear the EOC bit
      ADC1->SR |= (0<<4);//}
}
}

