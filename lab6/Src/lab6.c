#include "main.h"
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"

void SystemClock_Config(void);

// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};

// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};

// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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

   // Turn on the GPIOC Peripheral clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;


  // Configure LED GPIOC Output pins for LED use
  // PC6 = red, PC8 = orange, PC9 = green, PC7 = blue on the STM32F072RBT6
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,      // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,              // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9
  // My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); // turn on green LED

  // Select a GPIO (ADC_IN10) pin to use as the ADC input
  //   by setting it to Analog with no pull-up
  // - Selected GPIOC Pin 0, which connects to ADC in channel 10
  initStr.Pin = GPIO_PIN_0;
  initStr.Mode = GPIO_MODE_ANALOG;
  initStr.Pull = GPIO_NOPULL;
  My_HAL_GPIO_Init(GPIOC, &initStr);

  // Enable the ADC peripheral clock in RCC register
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Set the 4th bit (16 bit) in the ADC Control Register 1
  // which sets the ADC to 8 bit resolution (10b)
  ADC1->CFGR1 |= ADC_CFGR1_RES_1;

  // Select channel 10 for ADC conversion scanning
  ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

  // Wait for ADEN an DMAEN be cleared
  while (((ADC1->CR & ADC_CR_ADEN) != 0) && ((ADC1->CFGR1 & ADC_CFGR1_DMAEN) != 0)){};

  // Set the ADC calibration bit
  ADC1->CR &= ~ADC_CR_ADCAL_Msk;
  ADC1->CR |= ADC_CR_ADCAL;

  // Wait for ADCAL to reset
  while((ADC1->CR & ADC_CR_ADCAL) != 0){}; // 0x80000000); // << ADC_CR_ADCAL_Pos) ){};

  uint32_t calibrationFactor = ADC1->DR;

  // Enable the ADC
  ADC1->CR |= ADC_CR_ADEN;

  // Wait for ADC ready ADRDY flag to be set
  while(((ADC1->ISR & ADC_ISR_ADRDY) != 1)){};

  // Set ADC to continuous mode
  ADC1->CFGR1 |= ADC_CFGR1_CONT;

  // Set the ADC Start bit
  ADC1->CR |= ADC_CR_ADSTART;

  uint32_t adcValue;
  uint32_t threshold = 255; // 8-bit max

  /************************************
  // Lab 6.1
  // Generating Waveforms with the DAC
  *************************************/

  // Enable the GPIOA clock in RCC to turn on GPIO Port A
   RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

   // Enable the DAC clock to turn on the DAC
   RCC->APB1ENR |= RCC_APB1ENR_DACEN;

  // Select PA4 as GPIO pin to use for DAC output
  // PA4 = DAC_OUT1
  initStr.Pin = GPIO_PIN_4;
  initStr.Mode = GPIO_MODE_ANALOG;
  initStr.Pull = GPIO_NOPULL;
  My_HAL_GPIO_Init(GPIOA, &initStr);

  // Set the DAC channel to software trigger mode
  // TSEL = 0x111
  DAC->CR |= (DAC_CR_TSEL1_0 | DAC_CR_TSEL1_1 |DAC_CR_TSEL1_2);

  // Enable the DAC channel
  DAC->CR |= DAC_CR_EN1;

  while (1)
  {
 
    for (int i = 0; i < sizeof(sawtooth_table); i++) {
    
      // Turn on LEDs accroding to the ADC's input voltage
      adcValue = ADC1->DR;

      if (adcValue > (20)){
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,SET);
      } else {
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,RESET);
      }

      if (adcValue > (1*(threshold / 3)-0)){
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,SET);
      } else {
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,RESET);
      }

      if (adcValue > (2*(threshold / 3)-20)){
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,SET);
      } else {
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7,RESET);
      }

      if (adcValue > (255-40)){
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,SET);
      } else {
        My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,RESET);
      }

      // Generate a waveform with the DAC converter by sending next value to DAC
      DAC->DHR8R1 = sine_table[i];      DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

      HAL_Delay(100); // Delay 1 ms
    }




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
