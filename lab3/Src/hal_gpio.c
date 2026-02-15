#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <stdbool.h>

void My_HAL_EXTI_Init(uint32_t intrptNum, bool enState){

  unsigned long temp;

  // Turn on the Interrupt
  temp = *((volatile unsigned long*)(0x40010400));   // EXTI base address = EXTI_IMR address
  temp = temp & ~(1 << intrptNum);                   // clear bit
  temp = temp |  (enState << intrptNum);             // Set bit
  *((volatile unsigned long*)(0x40010400)) = temp;

  // Set to Interrupt to rising edge
  temp = *((volatile unsigned long*)(0x40010408));   // EXTI_RTSR address
  temp = temp & ~(1 << intrptNum);                   // clear bit
  temp = temp |  (enState << intrptNum);             // Set bit
  *((volatile unsigned long*)(0x40010408)) = temp;

  // NVIC->IP[EXTI_LINE_0] = 0x01;
}


void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{

  uint32_t position = 0x00u;
  uint32_t currentGPIO;
  uint32_t temp;

  // Make sure valid parameters have been sent
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

  // Configure the GPIO ports on this GPIOx port
  while (((GPIO_Init->Pin) >> position) != 0x00u)
  {

    // current GPIO to be worked on
    currentGPIO = (GPIO_Init->Pin) & (1 << position);

    // if current GPIO pin is set
    if (currentGPIO != 0x00u)
    {
      //Set the Pin Mode (Input=00, GPIO01, Alternate Function=10, Analog Mode=11)
      temp = GPIOx->MODER;
      temp &= ~(0x03u << (position * 2u));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2u));
      GPIOx->MODER = temp;

      // Set GPIO Output Type to output push-pull=0, or ouput open-drain=1
      if((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT){
        // Set the GPIO Output Type
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
      }

      // Set pull-up, pull-down or no-pull-down if not Analog Mode
      if((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG){
        // Set the pin to either pull-up, pull-down, or no pull-up/down
        temp = GPIOx->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2u));
        temp |= ((GPIO_Init->Pull) << (position * 2u));
        GPIOx->PUPDR = temp;
      }

      // Set the Port Speed if pin is Ouput or Analog Mode
      if(((GPIO_Init->Mode & GPIO_MODE) == MODE_AF) | ((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT)){
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2u));
        temp |= (GPIO_Init->Speed << (position * 2u));
        GPIOx->OSPEEDR = temp;
      }

    }

    position++;

  }

  // // Hard coded GPIO configuration for use with Lab 1
  // // Configure the mode for GPIOC Pins 6, 7, 8, 9 as Output pins, which is 01
  // // Pin 6 = Red LED    (MODER pin13,12), Pin 7 = Blue LED (MODER pin15,14)
  // // Pin 8 = Orange LED (MODER pin17,16), Pin 9 = Green LED (MODER pin19,18)
  // GPIOC->MODER |= (1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18);
  // GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
  // // Configure GPIOC Pins 6, 7, 8, 0 as output type as push-pull (set to 0)
  // GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
  // // Configure GPIOC Pins 6, 7, 8, 9 to low speed (x0)
  // GPIOC->OSPEEDR &= ~((1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18));
  // // Configure GPIOC Pins 6, 7, 8, 9 to no pull-up/down (00)
  // GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) |
  //                   (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));
  // // Configure PA0 (User Button) as input, low speed, pull-down
  // // Pin0 of Port A is connected to the User Push-Button
  // GPIOA->MODER   &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
  // // Configure to low speed
  // GPIOA->OSPEEDR &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
  // // Set the Pull-Up/Down Register bits to Pull-Down (10)
  // // Set 1s
  // GPIOA->PUPDR   |=  ((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
  // // // Set 0s
  // GPIOA->PUPDR   &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));

}


/*
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/


GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState bitstatus;

  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
  {
    bitstatus = GPIO_PIN_SET;
  }
  else
  {
    bitstatus = GPIO_PIN_RESET;
  }
  return bitstatus;
}



void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{

    if (PinState != GPIO_PIN_RESET)
    {
        GPIOx->BSRR = (uint32_t)GPIO_Pin;
    }
    else
    {
        GPIOx->BRR = (uint32_t)GPIO_Pin;
    }

}



void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{

    int32_t output = GPIOx->ODR;

    // Toggle GPIO_Pin bits
    GPIOx->BSRR = ((output & GPIO_Pin) << 16) | (~output & GPIO_Pin);

}

