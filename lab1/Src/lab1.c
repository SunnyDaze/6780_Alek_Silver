#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"
#include "assert.h"

void SystemClock_Config(void);

int main(void) {
    HAL_Init(); //Reset of all peripherals, init the Flash and Systick
  SystemClock_Config(); //Configure the system clock

  /* This example uses HAL library calls to control
     the GPIOC peripheral.  You'll be redoing this code
     with hardware register access. */

    // HAL_RCC_GPIOC_CLK_ENABLE(); // previous way of setting the clock
    HAL_RCC_GPIOC_CLK_Enable(); //Enable the GPIOC clock in the RCC

    // check that the GPIOC clock bit is actually enabled
    assert((RCC->AHBENR && (1 << 19)) == 1);

  // Set up a configuration struct to pass to the initialize function
  GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,     // Mode  - GPIOx_OTYPER
                              GPIO_SPEED_FREQ_LOW,     // Speed - GPIOx_OSPEEDR
                              GPIO_NOPULL};            // Pull  - GPIOx_PUPDR

  // HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9
  assert((GPIOC->MODER   & 0x000F0000) == 0x00050000); // check that GPIO 8 and 9 are set to output
  assert((GPIOC->OTYPER  & 0x00000300) == 0x00000000); // check that GPIO 8 and 9 are push-pull
  assert((GPIOC->OSPEEDR & 0x00050000) == 0x00000000); // check that GPIO 8 and 9 are speed medium
  assert((GPIOC->PUPDR   & 0x000F0000) == 0x00000000); // check that GPIO 8 and 9 are no pull-up

  // Loop continuously sensing Push-Button presses,and toggle turning on LEDs
  uint32_t debouncer = 0;
  int down = 0;
  uint16_t led_pin = GPIO_PIN_6;
  while(1) {
    debouncer = (debouncer << 1); // Always shift every loop iteration
                                  // If no button push, all 1s shift out
                                  //   leaving a register of zeros

    int pushbutton = My_HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if ((pushbutton) == 1){   // If input signal is set high
      debouncer |= 0x01;  // Set lowest bit of bit-vector      
    }

    if (debouncer == 0xFFFFFFFF){
      // This code triggers repeatedly when button is high
      if (down == 0){
        My_HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_RESET);
        if (led_pin == GPIO_PIN_9){
          led_pin = GPIO_PIN_6;
        }
        else{
          led_pin = (led_pin << 1);  // shift to next LED
        }
        My_HAL_GPIO_WritePin(GPIOC, led_pin, GPIO_PIN_SET);
        down = 1;
        debouncer |= 0x00;
      }
    }
    if (debouncer == 0x00000000){
      // This code trigger repeatedly when button is steady low
      down = 0;  // button no longer down, ready to detect next down push
    }
    if (debouncer == 0x7FFFFFFFF){
      // This code triggers only once when transitioning to steady state
    }
    // When button is bouncing the bit vector value is random since bits are set when
    // the button is high and not when it bounces low
  }

  // Loop LEDs flashing back and forth
  // This code is never reached when the push-button code above is in forever loop
  // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // Start PC* high
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC* high
  assert((GPIOC->ODR & GPIO_PIN_6) == GPIO_PIN_6);    // verify that Pin 8 gets sets
  while (1) {
    HAL_Delay(200); //Delay 200ms
    // Toggle the output of both PC8 and PC9
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7);
    
  }
}

void HAL_RCC_GPIOC_CLK_Enable(void){
  
  // Enable the GPIOA Clock (for User Pushbutton)
  SET_BIT(RCC->AHBENR, (1 << 17)); // RCC_AHBENR_GPIOAEN);

  // Enable the GPIOC Clock (for LEDs)
  SET_BIT(RCC->AHBENR, (1 << 19)); // RCC_AHBENR_GPIOCEN);

}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add their own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
