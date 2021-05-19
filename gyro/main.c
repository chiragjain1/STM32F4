#include <__cross_studio_io.h>
#include <stm32f407xx.h>

float funct_val,intrpt_val,start_pt,angle,improv,error,error1;
void usart3_init();
void USART3_receive();
int map_value(long val, long min1 , long max1, long min2, long max2);
void improv_gyro();

void main(void)

{
   usart3_init();
   USART3_receive();
   start_pt = funct_val;
   while(1)
   {
    USART3_receive();
   
  //  if(funct_val < start_pt)
  //  angle = funct_val - start_pt;
    improv_gyro();
    
   }
}

void usart3_init()
{
    RCC->APB1ENR |= (1<<18);     //USART2
    RCC->AHB1ENR |= (1<<1);     //GPIO B
    GPIOB->MODER |= (1<<23) | (0<<22) | (1<<21) | (0<<20);    //Alternate function on 10,11
    GPIOB->AFR[1] |= (1<<12) | (1<<13) | (1<<14) | (1<<10) | (1<<9) | (1<<8);
    GPIOB->OSPEEDR |= (1<<23) | (1<<22);
    GPIOB->PUPDR |=(1<<22);

    USART3->CR1 |= (1<<13);		//(Usart Enable) 
    USART3->CR2 &= ~(1<<13) & ~(1<<12);		// 1 Stop bit
   // USART3->BRR |= (1<<7)|(1<<1)|(1<<0)|(1<<3);//FOR 115200
   USART3->BRR = (1<<0)|(1<<1)|(1<<7)|(1<<9)|(1<<10);//683 FOR 9600
    USART3->CR1 |=(1<<2)|(1<<5); 	//Receive,Receive Interrupt
  //  NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_receive()
  {
    
    while(!(USART3->SR & (1<<5)));
   // while(!(USART6->SR & (1<<6)));
    USART3->SR &= ~(1<<5);//rxne
    USART3->SR &= ~(1<<6);//tc
 //   tfmini_new =  USART6->DR;
     funct_val= USART3->DR;
     funct_val = map_value(funct_val,0,200,-127,127);
  }

void improv_gyro()
{
  float low_lim,up_lim;

   angle = funct_val - start_pt;
    if(angle<0)
	{
		angle = 255 + angle;
               if(angle>127)
                angle = angle - 255; 
	}
 
  low_lim = 127 - start_pt;
  up_lim = -127 - start_pt;
  improv = map_value(angle,low_lim,up_lim,-127,127);
 // improv = 
 error = improv;
    error1 = (error + start_pt);
}  

int map_value(long val, long min1 , long max1, long min2, long max2)
{
	return (val - min1) * (max2 - min2) / (max1 - min1) + min2;
}