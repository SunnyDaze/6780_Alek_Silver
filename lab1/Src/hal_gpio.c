#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
    assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

    // Configure the mode for GPIOC Pins 6, 7, 8, 9 as Output pins, which is 01
    // Pin 6 = Red LED    (MODER pin13,12), Pin 7 = Blue LED (MODER pin15,14)
    // Pin 8 = Orange LED (MODER pin17,16), Pin 9 = Green LED (MODER pin19,18)
    GPIOC->MODER |= (1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18);
    GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));

    // Configure GPIOC Pins 6, 7, 8, 0 as output type as push-pull (set to 0)
    GPIOC->OTYPER &= ~((1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));

    // Configure GPIOC Pins 6, 7, 8, 9 to low speed (x0)
    GPIOC->OSPEEDR &= ~((1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18));

    // Configure GPIOC Pins 6, 7, 8, 9 to no pull-up/down (00)
    GPIOC->PUPDR &= ~((1 << 12) | (1 << 13) | (1 << 14) | (1 << 15) |
                      (1 << 16) | (1 << 17) | (1 << 18) | (1 << 19));

    // Configure PA0 (User Button) as input, low speed, pull-down
    // Pin0 of Port A is connected to the User Push-Button
    GPIOA->MODER   &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
    GPIOA->OSPEEDR &= ~((1 << 0) | (1 << 1));  // clear bits 1 and 0
    // Set the Pull-Up/Down Register bits to Pull-Down (10)
    // Set 1s
    GPIOA->PUPDR   |=  ((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));
    // Set 0s
    GPIOA->PUPDR   &= ~((1 << 12) | (1 << 14) | (1 << 16) | (1 << 18));

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

