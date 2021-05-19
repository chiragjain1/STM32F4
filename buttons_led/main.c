#include <__cross_studio_io.h>
#include <stm32f407xx.h>

void main(void)
{
  RCC->AHB1ENR |= (1<<0)|(1<<1);//GPIOA AND GPIOB ENABLE
  GPIOA->MODER = 0X00005555;//General purpose output mode
  GPIOB->MODER = 0X00000000;//INPUT MODE[BUTTON]
  GPIOB->PUPDR = 0XAAAAAAAA;
  GPIOA->ODR = 0X00;

  while(1)
  {
    GPIOA->ODR = GPIOB->IDR;
  }
}
