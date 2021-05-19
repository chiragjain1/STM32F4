/*
AUTHOR:CHIRAG

This is for interrupt line zero and one
SYSCFG is used for telling the system that interrupt is enabled
NVIC_ENABLEIRQ is used for global interrupt enable[similar to AVR line]
EXTI IS the external interrupt register[event is not used,interrupt is used]
GPIO is usual one used.B is for led,GPIOA is used for button
*/

#include <__cross_studio_io.h>
#include <stm32f407xx.h>

void interrupt_zero_init(void);
void interrupt_one_init(void);
void gpio_init(void);

void main(void)
{
   interrupt_zero_init();
   interrupt_one_init();
   gpio_init();
}

void EXTI1_IRQHandler(void)
{
 GPIOB->ODR = (1<<0);
 EXTI->PR = (1<<1);//clear the interrupt request
 }

 void EXTI0_IRQHandler(void)
{
 GPIOB->ODR = (1<<1);
 EXTI->PR = (1<<0);//clear the interrupt request
 }

void interrupt_zero_init()
{
    SYSCFG->EXTICR[0] |= (1<<0);//These bits are written by software to select the source input for the EXTIx external interrupt.[PAGE 291]

    NVIC_EnableIRQ(EXTI0_IRQn);//ENABLE GLOBAL INTERRUPTS ON LINE 0
    
    EXTI->IMR |= (1<<0);    //Interrupt request from line x is not masked
    EXTI->RTSR |= (1<<0);   //Rising trigger enabled (for Event and Interrupt) for input line
}

void interrupt_one_init()
{
    SYSCFG->EXTICR[0] |= (1<<4);//These bits are written by software to select the source input for the EXTIx external interrupt.[PAGE 291]

    NVIC_EnableIRQ(EXTI1_IRQn);//ENABLE GLOBAL INTERRUPTS ON LINE 0
    
    EXTI->IMR |=  (1<<1);    //Interrupt request from line x is not masked
    EXTI->RTSR |= (1<<1);   //Rising trigger enabled (for Event and Interrupt) for input line
}

void gpio_init()
{
    RCC->AHB1ENR |= (1<<0)|(1<<1);//PORT A AND B

    GPIOB->MODER = 0X00005555; //GPIO_B OUTPUT MODE
    GPIOA->MODER = 0X00;       //GPIO_A INPUT MODE

    GPIOB->ODR = 0X00;
}