#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>

void My_HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
    assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

    // Configure GPIOC Pins 6, 7, 8, 9 as Output pins, which is 01
    // Pin 6 = Red LED    (MODER pin13,12), Pin 7 = Blue LED (MODER pin15,14)
    // Pin 8 = Orange LED (MODER pin17,16), Pin 9 = Green LED (MODER pin19,18)
    GPIOC->MODER |= (1 << 12) | ( 1 << 14) | ( 1 << 16) | ( 1 << 18);
    GPIOC->MODER &= ~((1 << 13) | (1 << 15) | (1 << 17) | (1 << 19));

    // Configure GPIOC Pins 6, 7, 8, 0 as

    // Configure GPIOC 

    // Configure GPIOC 



}


/*
void My_HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
}
*/

/*
GPIO_PinState My_HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    return -1;
}
*/

/*
void My_HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
}
*/

/*
void My_HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
}
*/
