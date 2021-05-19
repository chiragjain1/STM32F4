/*
AUTHOR : CHIRAG
REFER TO 610 OF MANUAL
THIS CODE GIVES PWM OUTPUT ON TWO CHANNELS[A6 AND B0]

*/


#include <__cross_studio_io.h>
#include <stm32f407xx.h>

volatile int pwm = 20;
volatile int pwm_two = 75;
void timer3_pwm_init(void);
void gpioab_led_init(void);

void main(void)
{
      NVIC_EnableIRQ(TIM3_IRQn);//enable timer interrupt
      timer3_pwm_init();//INITIALIZE PWM OF TIMER 
      gpioab_led_init();//OUTPUT IS ON A6 and B0
}

void TIM3_IRQHandler(void)
{
 
    TIM3->CCR1 = pwm;          //the pwm value is given to the timer
    TIM3->CCR3 = pwm_two; 
    TIM3->SR &= ~(1<<3);       //interrupt flas is cleared
    TIM3->SR &= ~(1<<1);       //interrupt flag is cleared
}

void timer3_pwm_init()
{

     RCC->APB1ENR |= (1<<1);//clock for TIMER
     TIM3->CR1 |= (1<<0)|(1<<7);//Bit 7 ARPE: Auto-reload preload enable//Bit 0 CEN: Counter enable
     TIM3->DIER |=(1<<0)|(1<<1)|(1<<3);//Bit 0 UIE: Update interrupt enable//Bit 1 CC1IE: Capture/Compare 1 interrupt enable//Bit 3 CC3IE: Capture/Compare 3 interrupt enable
     TIM3->CCMR1 = 0x0068;//THE OC1M IS SET AS : 110: PWM mode 1//OC1PE IS SET AS ONE
     TIM3->CCMR2 = 0X0068;//THE OC1M IS SET AS : 110: PWM mode 1//OC1PE IS SET AS ONE   
     TIM3->CCER = 0x0101;//CC1E: Capture/Compare 1 output enable.//CC3E: Capture/Compare 3 output enable.
     TIM3->PSC = 320;       //prescaler for timer
     TIM3->ARR = 1000;    //Maximum pwm value
     TIM3->CCR1 = 00;     //pwm value that will be given
     TIM3->CCR3 = 00;     //pwm value that will be given

}

void gpioab_led_init()
{
      RCC->AHB1ENR = (1<<0)|(1<<1);//clock for GPIOA
      GPIOA->AFR[0] |= (1<<25);//AF2 for GPIOA6
      GPIOB->AFR[0] |= (1<<1);//AF2 FOR GPIOB0
      GPIOA->MODER |= (1<<13);//ALTERNATE FUNCTION ON A6
      GPIOB->MODER |= (1<<1);
}