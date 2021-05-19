#include <__cross_studio_io.h>
#include <stm32f407xx.h>

double count=0, flag=0, edge=0, value=0, distance=0, output=0, counter=0;
void delay (int us);
void trigger();
void
main(void)
 {
    RCC->AHB1ENR |= (1<<3); //Port D clock enable

  RCC->APB1ENR |= (1<<0); //Timer 2 clock enable
   RCC->APB2ENR |= (1<<14); //SYSCFG clock enable
  //SYSCFIG_EXTICR1 is already set at 0 so PA0 is selected for external interrupt
  
   GPIOD->MODER &= ~((1<<0) | (1<<1)); //Pin D0 set as input
   GPIOD->MODER |= (1<<4);//D2 OUTPUT  
  
  SYSCFG->EXTICR[0] |= (1<<0)|(1<<1);

  EXTI->RTSR |= (1<<0); // Rising edge triggered
  EXTI->FTSR |= (1<<0); // Falling edge triggered  
  EXTI->IMR = (1<<0); //Interrupt enable

int counterdelay;
  //edge = 200;
  TIM2->PSC = 1280; // Prescalar setup
  TIM2->ARR = 65535; // Auto reload value
  NVIC_EnableIRQ(EXTI0_IRQn);

  trigger();
  while(1)
  {
    if(flag==0)
    {
      trigger();
    }
    output = distance;
    delay(65);
  }
}

void delay (int us)
{
  //counterdelay = 100;
  int counter = 10*us;
  while (counter--);  
}

void trigger()
{
  
  GPIOD->ODR &= ~(1<<2); //Pin D low
  delay(10);
  GPIOD->ODR |= (1<<2); //Pin D2/TRIG set high
  delay(25);
  GPIOD->ODR &= ~(1<<2); // Pin D2 low generating a trigger
//counter = 1111;
  
 // GPIOB->MODER &= ~((1<<0) | (1<<1)); //Pin A0 set as input
}

void EXTI0_IRQHandler(void)
{
  edge++;
  counter++;
  if (edge == 1)
  {
    TIM2->CNT = 0;
    TIM2->CR1 |= (1<<0); // Timer enable
    flag = 1;
  }
  else
  {
    count = TIM2->CNT;
    TIM2->CNT = 0;
    TIM2->CR1 &= ~(1<<0); //Timer stop
    flag = 0;
    edge = 0;
    value = (count/16000);
    distance = value*171.500*128;
  }
  EXTI->PR = (1<<0);
}