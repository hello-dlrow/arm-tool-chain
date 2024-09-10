#include <stdint.h>

#define PERIPHERAL_BASE (0x40000000U)

#define AHB_BASE (PERIPHERAL_BASE + 0x18000U)
#define RCC_BASE (AHB_BASE + 0x9000U)

#define APB2_BASE (PERIPHERAL_BASE + 0x10000U)
#define GPIOB_BASE (APB2_BASE + 0xc00U)

#define RCC_APB2ENR_OFFSET (0x18U)
#define RCC_APB2ENR ((volatile uint32_t*) (RCC_BASE + RCC_APB2ENR_OFFSET))
#define RCC_AHBENR_GPIOBEN (0x03U)

#define GPIO_MODER_OFFSET (0x00U)
#define GPIOB_MODER ((volatile uint32_t*) (GPIOB_BASE + GPIO_MODER_OFFSET))
#define GPIO_MODER_MODER5 (00U) //bit[3:0] = 0001 push-pull
#define GPIO_ODR_OFFSET (0x0cU)
#define GPIOB_ODR ((volatile uint32_t*) (GPIOB_BASE + GPIO_ODR_OFFSET))

#define LED_PIN 0
 
void main(void)
{
  *RCC_APB2ENR |= (1 << RCC_AHBENR_GPIOBEN);

  // do two dummy reads after enabling the peripheral clock, as per the errata
  volatile uint32_t dummy;
  dummy = *(RCC_APB2ENR);
  dummy = *(RCC_APB2ENR);

  *GPIOB_MODER &= ~(0xf);

  *GPIOB_MODER |= (1 << GPIO_MODER_MODER5);
  
  while(1)
  {
    *GPIOB_ODR ^= (1 << LED_PIN);
    for (uint32_t i = 0; i < 1000000; i++);
  }

}
 
