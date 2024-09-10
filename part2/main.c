#include <stdint.h>
#include "stm32f1xx.h"

#define LED_PIN 0

void clock_init(void){

//reset RCC_CR and turn off HSE
RCC->CR = 0x00000083;
RCC->CR &= ~(RCC_CR_HSEON_Msk);


//turn on HSE
RCC->CR |= RCC_CR_HSEON_Msk;
while (!(RCC->CR & RCC_CR_HSERDY_Msk));


        //-----------------------------------------------------------
        // 使能FLASH 预存取缓冲区 */
        FLASH->ACR |= FLASH_ACR_PRFTBE;

        // SYSCLK周期与闪存访问时间的比例设置，这里统一设置成2
        // 设置成2的时候，SYSCLK低于48M也可以工作，如果设置成0或者1的时候，
        // 如果配置的SYSCLK超出了范围的话，则会进入硬件错误，程序就死了
        // 0：0 < SYSCLK <= 24M
        // 1：24< SYSCLK <= 48M
        // 2：48< SYSCLK <= 72M */
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
        //------------------------------------------------------------



//reset RCC_CFGR
RCC->CFGR = 0x00000000;

// Clear PLLSRC, PLLXTPRE, PLLMULL bits
RCC->CFGR &= ~(RCC_CFGR_PLLSRC |
                  RCC_CFGR_PLLXTPRE |
                  RCC_CFGR_PLLMULL);

//HSX = (/2 *9) 72MHz
RCC->CFGR |= (RCC_CFGR_PLLSRC |
              RCC_CFGR_PLLXTPRE |
              RCC_CFGR_PLLMULL16);


// Set APB1 prescaler to 2
RCC->CFGR |= (0b100 << RCC_CFGR_PPRE1_Pos);



// Enable PLL and wait for ready
RCC->CR |= RCC_CR_PLLON;
while (! (RCC->CR & RCC_CR_PLLRDY));



// Select PLL output as system clock
RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
while (! (RCC->CFGR & RCC_CFGR_SWS_PLL));

}







uint32_t ticks;

void systick_handler()
{
  ticks++;
}

void delay_ms(uint32_t milliseconds)
{
  uint32_t start = ticks;
  uint32_t end = start + milliseconds;

  if (end < start) // handle overflow
  {
    while (ticks > start); // wait for ticks to wrap around to zero
  }

  while (ticks < end);
}
  

void main(void)
{ 
  clock_init();
  SysTick_Config(72000);
  __enable_irq();


  RCC->APB2ENR |= (1 << RCC_APB2ENR_IOPBEN_Pos);
  
  // do two dummy reads after enabling the peripheral clock, as per the errata
  volatile uint32_t dummy;
  dummy = RCC->AHBENR;
  dummy = RCC->AHBENR;

  GPIOB->CRL &= ~(0xf);
  GPIOB->CRL |= (1 <<  GPIO_CRL_MODE0_Pos);


  while(1)
  {
    GPIOB->ODR ^= (1 << LED_PIN);
    delay_ms(500);
  }
}