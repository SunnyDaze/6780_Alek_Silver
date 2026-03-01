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

  // Turn on GPIOB and GPIOC Peripheral clocks
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure LED GPIOC Output pins for LED use
  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,      // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,              // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9

  /********************************************
  // Set up the gyroscope chip for use with I2C2
  // instead of default SPI mode
  ********************************************/

  // Configure PB11
  initStr.Pin = GPIO_PIN_11,
  initStr.Mode = GPIO_MODE_AF_OD,
  initStr.Pull = GPIO_NOPULL,
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
    // Initializes PB11
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set PortB Pin11 to Alternate Function 1 (AF1)
  // which connects it to the STM32 I2C_SDA pin
  // See page 162 & 163/1017 in periph. ref. manual
  // AF1 = (0x01), pin 11 = postion 3 in high register [1]
  GPIOC->AFR[1] |= (0x01 << GPIO_AFRH_AFRH3_Pos);


  // Set GPIOB Pins 13 alternate function mode
  initStr.Pin = GPIO_PIN_13;
  initStr.Mode = GPIO_MODE_AF_OD;
  initStr.Pull = GPIO_NOPULL;
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
  // Initializes pin PB13
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set Port B pin 13 to 
  // which connects it to the STM32 I2C_SCL pin
  // AF1 = (0x05), pin 13 = postion 5 in high register [1]
  GPIOC->AFR[1] |= (0x05 << GPIO_AFRH_AFRH5_Pos);

  // Configure GPIOB Pin14
  initStr.Pin = GPIO_PIN_14;
  initStr.Mode = GPIO_MODE_OUTPUT_PP;
  initStr.Pull = GPIO_NOPULL;
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
  // Initializes pin PB14
  HAL_GPIO_Init(GPIOB, &initStr);
  // Set initial state to high
  // This is tied to SA0 (pin 4) on the gryoscope chip
  // setting the slave address to 1101001b or 0x69
  My_HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);

  // Configure GPIOC Pin0
  initStr.Pin = GPIO_PIN_0;
  initStr.Mode = GPIO_MODE_OUTPUT_PP;
  initStr.Pull = GPIO_NOPULL;
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
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
  //               PRESC = 0xB | SCLL = 0x13 | SCLH = 0xF | SDADEL = 0x2 | SCLDEL = 0x4 
  I2C2->TIMINGR = (0xB << 28   | 0x13 << 0   | 0xF << 8   | 0x2 << 16    | 0x4 << 20   );

  // Enable I2C2 in the I2C2_CR1 register by setting PE (bit0) to 1
  I2C2->CR1 |= (1 << I2C_CR1_PE_Pos);

  // Set the timing register TIMINGR to use
  // 100kHz standard-mode I2C
  // See pages 666 & 691/1017 in periph. ref. manual
  // Reset value: 0x0000 0000
  //               PRESC = 0xB | SCLL = 0x13 | SCLH = 0xF | SDADEL = 0x2 | SCLDEL = 0x4 
  I2C2->TIMINGR = (0xB << 28   | 0x13 << 0   | 0xF << 8   | 0x2 << 16    | 0x4 << 20   );

  // Enable I2C2 in the I2C2_CR1 register by setting PE (bit0) to 1
  I2C2->CR1 |= (1 << I2C_CR1_PE_Pos);

  /********************************************
  // Read the WHO_AM_I register on the 
  // gyroscope I2C peripheral
  // Green LED of correct, RED if wrong
  ********************************************/

  // Set gyroscope slave addres to 0x69 ... 
  //  ... PB14 was already set high above
  // PB14 is tied to SA0 (pin 4) on the gryoscope chip
  // setting PB14 high set slave addr to 1101001b or 0x69

  // set number of bytes to transmit =1
  I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;  // clear NBYTES
  I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos); //(0x1 << 16)

  // set RD_WRN to indicate a write (0=write)
  I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk; // clear RD_WRN
  I2C2->CR2 |= (0 << I2C_CR2_RD_WRN_Pos); //(0 << 10)

  // set the start bit
  I2C2->CR2 |= (1 << I2C_CR2_START_Pos); // set Start bit
  
  // wait until either the TXIS or NACKF flags are set
  if (!(I2C2->CR1 & I2C_CR1_TXIE_Msk) |
      !(I2C2->CR1 & I2C_CR1_NACKIE_Msk)){
    
    // TXIS flag is set, continue
    if(!(I2C2->CR1 & I2C_CR1_TXIE_Msk)){

      // Write the Address of the gyroscope's 
      // "WHO_AM_I" register (0x0F)
      // into the I2C transmit register TXRD
      I2C2->TXDR = 0x0F;
      
    }
  };

  // Wait until the TC (Transfer Complete) flag is set
  while(!(I2C2->ISR & I2C_ISR_TC_Msk)){
    // Do nothing, just wait
  }

  // Reload the CR2 registers with same parameters as before
  // but set RD_WRN to indicate a READ operation (1=read)
  I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;       // clear NBYTES
  I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos); //(0x1 << 16)
  I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;       // clear RD_WRN
  I2C2->CR2 |= (1 << I2C_CR2_RD_WRN_Pos); //(1 << 10)
  I2C2->CR2 |= (1 << I2C_CR2_START_Pos);  // set Start bit

  // wait until either the RXNE or NACKF flags are set
  //  - Continue if the RXNE flag is set
  uint32_t readbyte = 0x0000;
  if (!(I2C2->ISR & I2C_ISR_RXNE_Msk) |
      !(I2C2->ISR & I2C_ISR_NACKF_Msk)){
    
    // TXIS flag is set, continue
    if(!(I2C2->ISR & I2C_ISR_RXNE_Msk)){

      // Write the Address of the gyroscope's 
      // "WHO_AM_I" register (0x0F)
      // into the I2C transmit register TXRD
      readbyte = I2C2->RXDR;
      
    }
  };

  // Wait until the TC (Transfer Complete) flag is set
  while(!(I2C2->ISR & I2C_ISR_TC_Msk)){
    // Do nothing, just wait
  }

  // Check the contents of the RXDR register to see
  //  if it matches the expected value of 0xD3
  if(readbyte == 0xD3){
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);   // green
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET); // red
  }  else{
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);   // red
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET); // green
  }

  // Set the stop bit in the CR2 register to release the I2C2 bus
  I2C2->CR2 |= I2C_CR2_STOP_Msk;

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