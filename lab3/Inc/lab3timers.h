#include <stdint.h>
#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_gpio.h>
#include <stdbool.h>

void TIM2_Init(void);
void TIM3_Init(void);
void Connect_LEDs_to_TIM3(uint16_t red_on_time, uint16_t blue_on_time);