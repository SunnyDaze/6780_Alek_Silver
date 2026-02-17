// #include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"
#include "assert.h"
#include "lab3timers.h"
#include "core_cm0.h"

void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  // Enable relevant clocks (Clock on port C for LEDs and Port A for push-button_)
  // HAL_RCC_GPIOA_CLK_Enable(); // Enable the GPIOC clock in the RCC for the Blue Push-button
  HAL_RCC_GPIOC_CLK_Enable(); // Enable the GPIOC clock in the RCC for the LEDs


  // Configure LED GPIOC Output pins for LED use
  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9, // Pins  - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,     // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,             // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9

  // Turn on Green LED (GPIOC Pin 9)
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  // Turn on Orange LED (GPIOC Pin 8)
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  // Initialize the timer2
  TIM2_Init();

  // Configure LED GPIOC Output pins for LED use
  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  initStr.Pin   = GPIO_PIN_6 | GPIO_PIN_7; // Pins  - GPIOx_MODER
  initStr.Mode  = GPIO_MODE_AF_PP;         // Mode  - GPIOx_OTYPER
  initStr.Pull  = GPIO_NOPULL;             // Pull  - GPIOx_PUPDR
  initStr.Speed = GPIO_SPEED_FREQ_LOW;     // Speed - GPIOx_OSPEEDR
  HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9

  // Initialize Timer 3 for PWM output to PC6 and PC7
  TIM3_Init();

  // Connect red and blue LED to TIM3 PWM outputs
  // First is red LED time, and 2nd is blue LED time in us
  Connect_LEDs_to_TIM3(750, 50);

  // Enable/start the timer 3
  TIM3->CR1 |= TIM_CR1_CEN;
  
  // values for forever loop that pulses the blue and red LEDs
  int time = 500;
  int delay = 1;

  while (1)
  {
    for (int i = 1; i <= time; i++){
      TIM3->CCR1 = i+(1250-time);    // PC6 red LED time off - TIM3->ARR max counter value
      TIM3->CCR2 = i;    // PC7 blue LED time on - TIM3->ARR max counter value

      HAL_Delay(delay);
    }
    HAL_Delay(400);
    for (int i = time; i >= 1; i--){
      TIM3->CCR1 = i+(1250-time);    // PC6 red LED time off - TIM3->ARR max counter value
      TIM3->CCR2 = i;    // PC7 blue LED time on - TIM3->ARR max counter value

      HAL_Delay(delay);
    }
    HAL_Delay(400);
  }

}


void TIM2_IRQHandler(void){

  // Toggle Green LED (GPIOC Pin 9)
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
  // Toggle Orange LED(GPIO 8)
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);

  // Clear the Update Interrupt Flag for Timer 2
  TIM2->SR &= ~TIM_SR_UIF;
  // Clear the Timer 2 Pendong Interrupt Request
  // Note: Clearing the Update Interrupt Flag seems to 
  // also clear the NVIC Pending IRQ
  NVIC_ClearPendingIRQ(TIM2_IRQn);

}


void HAL_RCC_GPIOA_CLK_Enable(void){
  // Enable the GPIOA Clock (for User Pushbutton)
  SET_BIT(RCC->AHBENR, (1 << 17)); // RCC_AHBENR_GPIOAEN);
}


void HAL_RCC_GPIOC_CLK_Enable(void){
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
