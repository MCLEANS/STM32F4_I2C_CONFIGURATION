#include "stm32f4xx.h"

int main(void) {

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  GPIOA->MODER |= GPIO_MODER_MODER7_0;
  GPIOA->MODER |= GPIO_MODER_MODER6_0;

  GPIOA->ODR |= GPIO_ODR_ODR_6;
  GPIOA->ODR &= ~GPIO_ODR_ODR_7;

  while(1){
    for(volatile int i = 0; i < 2000000; i++){}
    GPIOA->ODR ^= (1<<6);
    GPIOA->ODR ^= (1<<7);

  }
}
