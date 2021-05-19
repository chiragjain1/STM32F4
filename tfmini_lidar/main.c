
#include <__cross_studio_io.h>
#include <stm32f407xx.h>

int dist=0; //actual distance measurements of LiDAR
int check=0; //save check value
int i=0,j=0;
int uart[9] ; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package
int tfmini=0,tfmini_new=0,news=0,flagnew=0;

void USART6_receive();
void USART6_init();
void lidar_value();
void main (void)
{
  RCC->APB2ENR |= (1<<14);
  USART6_init();
 // USART6_init();//LIDAR
  while(1)
  {
     lidar_value();

  }
}

void USART6_init()
{
    
    RCC->APB2ENR |= (1<<5);//usart6 clock
    RCC->AHB1ENR |= (1<<2);//GPIOC CLOCK

    GPIOC->MODER |= (1<<13)|(1<<15);//ALTERNATE FUNCTION FOR BIT 6 AND 7
    GPIOC->AFR[0] |= (1<<27)|(1<<31);//AF8 ON BIT 6 AND 7 

    USART6->CR1 |= (1<<13);//ENABLE USART
    //  USART6->BRR |= (1<<7)|(1<<1)|(1<<0)|(1<<3);//FOR 115200
    USART6->BRR = 0X02D9;//PLL NO OVER8 84 MHZ
    USART6->CR1 |= (1<<2);
    USART6->DR = 00;
}

 void USART6_receive()
  {    
    while(!(USART6->SR & (1<<5)));
    USART6->SR &= ~(1<<5);//rxne
    USART6->SR &= ~(1<<6);//tc
     tfmini = USART6->DR;
  }

  void lidar_value()
  {
    USART6_receive();

         if(tfmini==HEADER)
         {
            uart[0] = HEADER;             
              USART6_receive();             
              if(tfmini == HEADER){
                uart[1] = HEADER;
                  for(j=2;j<9;j++)
                  {
                    USART6_receive();
                    uart[j]=tfmini;
                  }
                  check =  uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

                  if (uart[8] == (check & 0xff ))
                  {
                //verify the received data as per protocol
                    dist = uart[2] + uart[3] * 256; //calculate distance value
                    
                  }
                  else
                  {
                    dist = uart[3] + uart[4]*256;
                    
                  }
              }
         }
  }