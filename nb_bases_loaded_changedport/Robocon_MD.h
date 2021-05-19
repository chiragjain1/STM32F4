#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#define MAXRETRY 2
#define Success 120
#define Fail 250

void uart4_init();
void transmit_MD(uint8_t);
void updatechecksum(uint8_t);
void clearchecksum();
uint8_t getchecksum();
void drive_M1(uint8_t,int);
void drive_M2(uint8_t,int);
void initTimer4();
int checktimeout();
void write_n__MD(uint8_t cnt, ... );
void init_MD();

uint8_t feedback=0;
uint16_t checksum=0;
int timeout=0;
int debug=0;

void init_MD()
{
  uart4_init();
 // transmit_MD_MD(0);
}

void uart4_init()
{
  RCC->APB1ENR |= (1<<19);
  RCC->AHB1ENR |= (1<<2);
  GPIOC->MODER |= (1<<23) | (1<<21);
  GPIOC->AFR[1] |= (1<<15) | (1<<11);
  GPIOC->OSPEEDR |= (1<<23) | (1<<22);
  GPIOC->PUPDR |=(1<<22);
  UART4->CR1 |= (1<<13);//|(1<<5) enable usart and interupt
  UART4->CR2 &= ~(1<<13) & ~(1<<12);//stop 1
  UART4->GTPR |=(1<<0);//prescalar
  UART4->BRR = 0x683;//over8=0 9600
  UART4->BRR = 0x1117;//over8=0 9600//pll
  //UART4->BRR |=(1<<11) | (1<<10) | (1<<8) |(1<<2) | (1<<1);//over8=1 9600
  UART4->CR1 |=(1<<3);// | (1<<2) enable receive
  //NVIC_EnableIRQ(UART4_IRQn);
}

void transmit_MD(uint8_t word)
{
  UART4->DR=word;
  while(!(UART4->SR & (1<<6)));
  UART4->SR&=~(1<<6);
}

/*void UART4_IRQHandler()
{
  UART4->SR &= ~(1<<5);//receive
  feedback=UART4->DR;
}*/

void updatechecksum(uint8_t data)
{
  checksum+=data;
}

void clearchecksum()
{
  checksum=0;
  feedback=0;
}

uint8_t getchecksum()
{
  while(checksum>0xFF)
  {
    uint8_t lsb=(0xFF&checksum);
    uint8_t msb=(0xFF&(checksum>>8));
    checksum= lsb+msb;
  }
  checksum=0xFF-checksum;
  return checksum;
}

void drive_M1(uint8_t address,int pwm)
{
  if(pwm<0)
    pwm=(int)(127+fabs(pwm));
  write_n__MD(4,4,address,30,pwm);
}

void drive_M2(uint8_t address,int pwm)
{
  if(pwm<0)
    pwm=(int)(127+fabs(pwm));
  write_n__MD(4,4,address,31,pwm);
}

void write_n__MD(uint8_t cnt, ... )
{
  init_MD();
  uint8_t trys=MAXRETRY;
  do
  {
    clearchecksum();
    va_list marker;
    va_start(marker,cnt);     
    for(uint8_t index=0;index<cnt;index++)
    {
      uint8_t data = va_arg(marker, int);
      updatechecksum(data);
      transmit_MD(data);
    }
    va_end(marker);              
    uint8_t checksum = getchecksum();
    transmit_MD(checksum);	
    //initTimer4();
    //while( feedback==0 && checktimeout()==0);
  }while(trys--);// && feedback==Fail);
}

void initTimer4()
{
  timeout=0;
  RCC->APB1ENR|=(1<<2);
  TIM4->CR1 |= (1 << 0);
  TIM4->DIER |= (1 << 0);
  TIM4->PSC = 0xFFFE; //
  TIM4->ARR = 122;
  TIM4->CNT=0;
  NVIC_EnableIRQ(TIM4_IRQn);
}

int checktimeout()
{
  return timeout;
}

void TIM4_IRQHandler() {
  TIM4->SR &= ~(1<<0);
  timeout=1;
}
