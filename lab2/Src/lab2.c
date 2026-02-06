#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"

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
  HAL_RCC_GPIOC_CLK_Enable(); //Enable the GPIOC clock in the RCC
  HAL_RCC_GPIOA_CLK_Enable(); //Enable the GPIOC clock in the RCC


  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,     // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,             // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR

  // Configure LED GPIOC Output pins for LED use
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9

  // Tiurn on Green LED (GPIOC Pin 9)
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC* high
  // assert((GPIOC->ODR & GPIO_PIN_6) == GPIO_PIN_6);    // verify that Pin 8 gets sets

  uint16_t led_count = 0x00u;

  while (1)
  {

    // Loop Blue LED (GPIOC Pin 6) to flash on and off
    // This indicates that the forever main loop is running
    HAL_Delay(600); //Delay 200ms
    // Toggle the output of both PC8 and PC9
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
    My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

  }
 
  return -1;
}

void HAL_RCC_GPIOC_CLK_Enable(void){
  
  // Enable the GPIOA Clock (for User Pushbutton)
  SET_BIT(RCC->AHBENR, (1 << 17)); // RCC_AHBENR_GPIOAEN);

}


void HAL_RCC_GPIOA_CLK_Enable(void){

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
