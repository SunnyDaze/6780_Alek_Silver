#include "main.h"
#include "stm32f0xx_hal.h"
#include "assert.h"
#include "lab3timers.h"
#include "hal_gpio.h"
#include "core_cm0.h"

// Initialize Timer 2
void TIM2_Init(void){

  // Enable Timer 2 and 3
  // Note: Timer 2 and 3 are on ABP1ENR
  // Note: Timer 1       is  on ABP2ENR
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure timer 2 to trigger an update event (UEV) at 4Hz
  uint32_t fCLK = 8000000u;  // 8Mhz system clock down to 1Khz for 1ms
  uint32_t fTarget2 = 4u;    // Desired Event Frequency (4Hz)
  uint32_t PSC2 = 7999u;     // prescale 8Mhz down to 1Khz for 1ms

  // Calculate values for Auto Reload Registers (ARRx)
  uint32_t ARR2 = (fCLK)/((PSC2+1)*fTarget2);  // should be 250

  // Load values and set up the timers Prescalers and AARs
  TIM2->PSC = PSC2; // 7999; // reduces 8Mhz by 8000 for a 1Khz clock
  TIM2->ARR = ARR2; // 250;  // count to 250ms for a 4Hz signal

  // Configure the timers to generate an interrupt on the UEV event
  TIM2->DIER |= TIM_DIER_UIE;
  // TIM3->DIER |= TIM_DIER_UIE;  // not using IRQ on this timer 3

  // Configure and enable/start the timer 2
  TIM2->CR1 |= TIM_CR1_CEN;
  // TIM3->CR1 |= TIM_CR1_CEN;

  // Set up timer's int handler in main code, and enable it in NVIC
  // need to #include "core_cm0.h" for NVIC_EnableIRQ to work on Cortex-M0
  NVIC_EnableIRQ(TIM2_IRQn);
  // NVIC_EnableIRQ(TIM3_IRQn); // not using IRQ on Timer 3

}


void TIM3_Init(void){

  ////////////////////////////////////////////////
  // Set up Timer 3
  ////////////////////////////////////////////////

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // Configure timer 3 to trigger an update event (UEV) at 800Hz
  uint32_t fCLK = 8000000u;  // 8Mhz system clock down to 1Khz for 1ms
  uint32_t fTarget3 = 800u;  // Desired Event Frequency (800Hz)
  uint32_t PSC3 = 7u;        // prescale 8Mhz down to 1Mhz for 1us

  // Calculate values for Auto Reload Registers (ARRx)
  uint32_t ARR3 = (fCLK)/((PSC3+1)*fTarget3);  // should be 1250

  // Load values and set up the timers Prescalers and AARs
  TIM3->PSC = PSC3; // 7;    // reduces 8Mhz by 8 for a 1Mhz clock (1us)
  TIM3->ARR = ARR3; // 1250; // count to 1250us for a 800Hz signal

  // enable output compare preload for both channels
  TIM3->CCMR1 |= ((1 << 11) | (1 << 3));

  // set CC1S to 00 (output mode)      // set CC2S to 00 (output mode)
  TIM3->CCMR1 &= ~((1 << 0) | (1 << 1) | (1 << 8) | (1 << 9)); //clear bits

  // // set OC2M to PWM1 (110b)
  TIM3->CCMR1 |= ((1 << 14) | (1 << 13)); // set 2 bits
  TIM3->CCMR1 &= ~(1 << 12);              // clear bit 
  // // set OC1M to PWM2 (111b)
  TIM3->CCMR1 |= ((1 << 6) | (1 << 5) | (1 << 4)); // set all 3 bits
  // TIM3->CCMR1 |= ((1 << 6) | (1 << 4)); // set 101 bits (force high)
  
  // Set output enable bits for capture/compare channels 1 & 2
  // TIM3->CCER |= ((1 << 0) | (1 << 4));  // Enable blue and red LEDs
  TIM3->CCER |= TIM_CCER_CC1E;  // Enable red LED
  TIM3->CCER |= TIM_CCER_CC2E;  // Enable blue LED

  // Set CCRx for both channels 1 and 2 to 20% of Timer3 ARR value
  // 20% of 1250 is 250
  // 40% = 500
  // 60% = 750
  // 50% = 526
  // 80% = 1000
  // 100% = 1250
  TIM3->CCR1 |= 250u;  // red LED time off - 65535 max counter value
  TIM3->CCR2 |= 250u;  // bue LED time on  - 65535 max counter value

}
