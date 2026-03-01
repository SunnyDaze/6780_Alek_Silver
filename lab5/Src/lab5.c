#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"

void SystemClock_Config(void);
void Error_Handler(void);

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

  /********************************************
  // Set up the gyroscope chip for use with I2C2
  // instead of default SPI mode
  ********************************************/
  
  // Turn on GPIOB and GPIOC Peripheral clocks
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure PB11
  GPIO_InitTypeDef initStr = {GPIO_PIN_11,             // Pin   - GPIOx_MODER
                              GPIO_MODE_AF_OD,        // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,            // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};  // Speed - GPIOx_OSPEEDR
    // Initializes PB11
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set PortB Pin11 to Alternate Function 1 (AF1)
  // which connects it to the STM32 I2C_SDA pin
  // See page 162 & 163/1017 in periph. ref. manual
  // AF1 = (0x01), pin 11 = postion 3 in high register [1]
  GPIOC->AFR[1] |= (0x01 << GPIO_AFRH_AFRH3_Pos);


  // Set GPIOB Pins 13 alternate function mode
  initStr.Pin = GPIO_PIN_13;            // Pins  - GPIOx_MODER
  initStr.Mode = GPIO_MODE_AF_OD;       // Mode  - GPIOx_OTYPER
  initStr.Pull = GPIO_NOPULL;           // Pull  - GPIOx_PUPDR
  initStr.Speed = GPIO_SPEED_FREQ_LOW;  // Speed - GPIOx_OSPEEDR
  // Initializes pin PB13
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set Port B pin 13 to 
  // which connects it to the STM32 I2C_SCL pin
  // AF1 = (0x05), pin 13 = postion 5 in high register [1]
  GPIOC->AFR[1] |= (0x05 << GPIO_AFRH_AFRH5_Pos);

  // Configure GPIOB Pin14
  initStr.Pin = GPIO_PIN_14;            // Pins  - GPIOx_MODER
  initStr.Mode = GPIO_MODE_OUTPUT_PP;   // Mode  - GPIOx_OTYPER
  initStr.Pull = GPIO_NOPULL;           // Pull  - GPIOx_PUPDR
  initStr.Speed = GPIO_SPEED_FREQ_LOW;  // Speed - GPIOx_OSPEEDR
  // Initializes pin PB14
  HAL_GPIO_Init(GPIOB, &initStr);
  // Set initial state to high
  My_HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);

  // Configure GPIOC Pin0
  initStr.Pin = GPIO_PIN_0;             // Pins  - GPIOx_MODER
  initStr.Mode = GPIO_MODE_OUTPUT_PP;   // Mode  - GPIOx_OTYPER
  initStr.Pull = GPIO_NOPULL;           // Pull  - GPIOx_PUPDR
  initStr.Speed = GPIO_SPEED_FREQ_LOW;  // Speed - GPIOx_OSPEEDR
  // Initializes pin PC0
  HAL_GPIO_Init(GPIOC, &initStr);
  // Set initial state to high
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);

  /********************************************
  // Set up the gyroscope I2C peripheral
  // using mostly the default values
  ********************************************/

  // Enable the I2C2 peripheral in the RCC
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Set the timing register TIMINGR to use
  // 100kHz standard-mode I2C
  // See pages 666 & 691/1017 in periph. ref. manual
  // Reset value: 0x0000 0000
  // I2C->TIMINGR = ()

  while (1)
  {
 
  }
  return -1;
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