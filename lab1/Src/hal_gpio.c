#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{

  uint32_t position = 0x00u;
  uint32_t iocurrent;
  uint32_t temp;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != 0x00u)
  {
    /* Get current io position */
    iocurrent = (GPIO_Init->Pin) & (1uL << position);

    if (iocurrent != 0x00u)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Output or Alternate function mode selection */
      if(((GPIO_Init->Mode & GPIO_MODE) == MODE_OUTPUT) ||
         ((GPIO_Init->Mode & GPIO_MODE) == MODE_AF))
      {
        /* Check the Speed parameter */
        assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (position * 2u));
        temp |= (GPIO_Init->Speed << (position * 2u));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & OUTPUT_TYPE) >> OUTPUT_TYPE_Pos) << position);
        GPIOx->OTYPER = temp;
      }
    }
      
    if((GPIO_Init->Mode & GPIO_MODE) != MODE_ANALOG)
    {
      /* Check the Pull parameter */
      assert_param(IS_GPIO_PULL(GPIO_Init->Pull));
      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPDR0 << (position * 2u));
      temp |= ((GPIO_Init->Pull) << (position * 2u));
      GPIOx->PUPDR = temp;
    }

    /* In case of Alternate function mode selection */
    if((GPIO_Init->Mode & GPIO_MODE) == MODE_AF)
    {
      /* Check the Alternate function parameters */
      assert_param(IS_GPIO_AF_INSTANCE(GPIOx));
      assert_param(IS_GPIO_AF(GPIO_Init->Alternate));
      /* Configure Alternate function mapped with the current IO */
      temp = GPIOx->AFR[position >> 3u];
      temp &= ~(0xFu << ((position & 0x07u) * 4u));
      temp |= ((GPIO_Init->Alternate) << ((position & 0x07u) * 4u));
      GPIOx->AFR[position >> 3u] = temp;
    }

    /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
    temp = GPIOx->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (position * 2u));
    temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2u));
    GPIOx->MODER = temp;

    /*--------------------- EXTI Mode Configuration ------------------------*/
    /* Configure the External Interrupt or event for the current IO */
    if((GPIO_Init->Mode & EXTI_MODE) != 0x00u)
    {
      /* Enable SYSCFG Clock */
      __HAL_RCC_SYSCFG_CLK_ENABLE();
      temp = SYSCFG->EXTICR[position >> 2u];
      temp &= ~(0x0FuL << (4u * (position & 0x03u)));
      temp |= (GPIO_GET_INDEX(GPIOx) << (4u * (position & 0x03u)));
      SYSCFG->EXTICR[position >> 2u] = temp;
      /* Clear Rising Falling edge configuration */
      temp = EXTI->RTSR;
      temp &= ~(iocurrent);
      if((GPIO_Init->Mode & TRIGGER_RISING) != 0x00u)
      {
        temp |= iocurrent;
      }
      EXTI->RTSR = temp;
      temp = EXTI->FTSR;
      temp &= ~(iocurrent);
      if((GPIO_Init->Mode & TRIGGER_FALLING) != 0x00u)
      {
        temp |= iocurrent;
      }
      EXTI->FTSR = temp;
      /* Clear EXTI line configuration */
      temp = EXTI->EMR;
      temp &= ~(iocurrent);
      if((GPIO_Init->Mode & EXTI_EVT) != 0x00u)
      {
        temp |= iocurrent;
      }
      EXTI->EMR = temp;
      temp = EXTI->IMR;
      temp &= ~(iocurrent);
      if((GPIO_Init->Mode & EXTI_IT) != 0x00u)
      {
        temp |= iocurrent;
      }
      EXTI->IMR = temp;
    }

    position++;

  }

//   // Hard coded GPIO configuration for use with Lab 1
//   // Configure the mode for GPIOC Pins 6, 7, 8, 9 as Output pins, which is 01
//   // Pin 6 = Red LED    (MODER pin13,12), Pin 7 = Blue LED (MODER pin15,14)
//   // Pin 8 = Orange LED (MODER pin17,16), Pin 9 = Green LED (MODER pin19,18)
//   GPIOC->MODER |= (1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18);
//   GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));  
//   // Configure GPIOC Pins 6, 7, 8, 0 as output type as push-pull (set to 0)
//   GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));  
//   // Configure GPIOC Pins 6, 7, 8, 9 to low speed (x0)
//   GPIOC->OSPEEDR &= ~((1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18));  
//   // Configure GPIOC Pins 6, 7, 8, 9 to no pull-up/down (00)
//   GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) |
//                     (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));  
//   // Configure PA0 (User Button) as input, low speed, pull-down
//   // Pin0 of Port A is connected to the User Push-Button
//   GPIOA->MODER   &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
//   GPIOA->OSPEEDR &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
//   // Set the Pull-Up/Down Register bits to Pull-Down (10)
//   // Set 1s
//   GPIOA->PUPDR   |=  ((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
//   // Set 0s
//   GPIOA->PUPDR   &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));

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

