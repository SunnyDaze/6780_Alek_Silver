#include "main.h"
#include "stm32f0xx_hal.h"
#include "assert.h"
#include "lab3timers.h"
#include "hal_gpio.h"
#include "core_cm0.h"

// Initialize Timer 2
void TIM2_Init(void){
  // Enable Timer 2
  // Note: Timer 2 and 3 are on ABP1ENR
  // Note: Timer 1       is  on ABP2ENR
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // Configure timer 2 to trigger an update event (UEV) at 4Hz
  uint32_t fCLK = 8000000u; // 8Mhz system clock down to 1Khz for 1ms
  uint32_t fTarget = 1000u;  // 1000 for how many per/second (Hz)
  uint32_t PSC = 7999u;      // prescale 8Mhz down to 1Khz for 1ms

  // Dividing the denominator by 1000 is the same as multiplying
  // the numerator by 1000, and helps avoid multiplying the 
  // numerator into a number too large for uint32
  uint32_t ARR = (fCLK)/((PSC+1)*fTarget/1000);

  TIM2->PSC = PSC; // 7999; // reduces 8Mhz by 8000 for a 1Khz clock
  TIM2->ARR = ARR; // 250;  // count to 250ms for a 4Hz signal

  // Configure the timer to generate an interrupt on the UEV event
  TIM2->DIER |= TIM_DIER_UIE;

  // Configure and enable/start the timer
  TIM2->CR1 |= TIM_CR1_CEN;

  // Set up timer's int handler in main code, and enable it in NVIC
  NVIC_EnableIRQ(TIM2_IRQn);

}
