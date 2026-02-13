#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"
#include "assert.h"

volatile uint16_t blue_led_count = 0x00u;
unsigned long temp;

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


  // Configure LED GPIOC Output pins for LED use
  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,     // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,             // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9

  // Turn on Green LED (GPIOC Pin 9)
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  // Turn on Red LED (GPIOC Pin 6) which wll be toggle in forever loop
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Start PC* high
  // assert((GPIOC->ODR & GPIO_PIN_6) == GPIO_PIN_6);    // verify that Pin 8 gets sets

  // Turn on Blue LED (GPIOC Pin 7) which will be toggle in SysTick
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Start PC* high

  // Set up a configuration for Pin 0 as pushbutton input w/pull-down
  initStr.Pin   = GPIO_PIN_0;           // Set the pin
  initStr.Mode  = GPIO_MODE_INPUT;      // Set the mode
  initStr.Pull  = GPIO_PULLDOWN;        // Set the pull-up/down
  initStr.Speed = GPIO_SPEED_FREQ_LOW;  // Set the speed
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9


  temp  = *((volatile unsigned long*)(0x40010400));   // EXTI base address = EXTI_IMR address
  assert(temp == 0x7F840000); // (temp & 0x00000001) == 0x00);
  // Enable EXTI interrupt 0
  My_HAL_EXTI_Init(EXTI_LINE_0, true);
  temp = *((volatile unsigned long*)(0x40010400));
  assert(temp == 0x7F840001); // (temp & 0x00000001) == 1u);


  HAL_RCC_SYSCFG_CLK_ENABLE(); //Enable the SYSCFG clock in the RCC

  temp  = *((volatile unsigned long*)(0x40010008));  // SYSCFG_EXTICR1 address
  assert(temp == 0x00000000); // (temp & 0x00000001) == 1u);
  // Set PA0 to connect to Interrupt 0
  SYSCFG->EXTICR[1] |= 0xFFFFFFF8;  //set lower 3 bits to 000b to connect it to PA0
  temp = *((volatile unsigned long*)(0x40010008));  // SYSCFG_EXTICR1 address
  temp &= 0x00000007;
  assert(temp == 0x00000000);

  // Enable the IRQ in the NVIC, and set it's priority
  NVIC_EnableIRQ(EXTI0_1_IRQn);
  NVIC_SetPriority(EXTI0_1_IRQn,1);



  // temp  = *((volatile unsigned long*)(0x40010008));  // SYSCFG_EXTICR1 address
  // temp = temp & ~(0xF << EXTI_LINE_0*4);        // clear all 4 bits
  // temp = temp |  (0x8 << EXTI_LINE_0*4);        // Set lower 3 bits to zeros
  // *((volatile unsigned long*)(0x40010400)) = temp;

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

void HAL_RCC_GPIOA_CLK_Enable(void){
  // Enable the GPIOA Clock (for User Pushbutton)
  SET_BIT(RCC->AHBENR, (1 << 17)); // RCC_AHBENR_GPIOAEN);
}


void HAL_RCC_GPIOC_CLK_Enable(void){
  // Enable the GPIOC Clock (for LEDs)
  SET_BIT(RCC->AHBENR, (1 << 19)); // RCC_AHBENR_GPIOCEN);
}

void HAL_RCC_SYSCFG_CLK_ENABLE(void){
  // RCC base address    = 0x40021000
  // RCC_APB2ENR address = 0x40021018  (offset 0x18)
  // Enable the SYSCFG Clock (for interrupts)
  SET_BIT(RCC->APB2ENR, (1 << 0)); // RCC_APB2ENR);
};

void EXTI0_1_IRQHandler(void){

  // Toggle Green LED (GPIOC Pin 9)
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

  // Toggle Orange LED(GPIO 8)
  My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
  
  
  // Clear pending
  // EXTI base = 0x4001 0400
  // EXTI_PR Address offset: 0x14
  temp = *((volatile unsigned long*)(0x40010414));   // EXTI base address = EXTI_IMR address
  temp = temp |  (1 << 0);    // Set bit
  *((volatile unsigned long*)(0x40010414)) = temp;

  // HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);

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
