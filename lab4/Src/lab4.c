#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.c"
#include "stdio.h"
#include "stdbool.h"
// #include "stm32f030x6.h"
// #include "stm32f030xc.h"

void SystemClock_Config(void);
void SendChar(char send_char);
void SendString(char str[]);

uint8_t volatile received = NULL;
bool volatile newdata = false;  // 1 if new data

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

  /***********************************************
     Initialize GPIOC clock and
     GPIOC Pins connected to the LEDs.
  ************************************************/

  // Turn on GPIOC Peripheral clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Configure LED GPIOC Output pins for LED use
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 |
                              GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,     // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,             // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  
  // Initializes pins PC8 & PC9
  My_HAL_GPIO_Init(GPIOC, &initStr);

  // Turn on all LEDs
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  
  
  /***********************************************
     Initialize GPIOC pin 4 and 5 to be
     set up as alternate functions
     connected to TX/RX of USART3
  ************************************************/

  // Set GPIOC Pins 4 and 5 into alternate function mode
  initStr.Pin = GPIO_PIN_4 | GPIO_PIN_5;  // Pins  - GPIOx_MODER
  initStr.Mode = GPIO_MODE_AF_PP;          // Mode  - GPIOx_OTYPER
  initStr.Pull = GPIO_NOPULL;              // Pull  - GPIOx_PUPDR
  initStr.Speed = GPIO_SPEED_FREQ_LOW;     // Speed - GPIOx_OSPEEDR
  // Initializes pins PC4 & PC5
  My_HAL_GPIO_Init(GPIOC, &initStr);

  // Set PortC_Pin4 and PortC_Pin5 to Alternate Function 1 (AF1)
  // which connects them to the USART3 RX/TX
  GPIOC->AFR[0] |= (0x01 << GPIO_AFRL_AFRL4_Pos);
  GPIOC->AFR[0] |= (0x01 << GPIO_AFRL_AFRL5_Pos);

  // Turn on USART3 Peripheral Clock in RCC_APB1ENR
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  // SET_BIT(RCC->AHB1ENR, (1 << 17));

  // Set the USART3 Baud rate to 115200 Baud
  uint32_t BAUD_RATE = 115200;
  uint32_t SYS_CLK = HAL_RCC_GetHCLKFreq();  // Should be 8000000 (8MHz)
  USART3->BRR = SYS_CLK / BAUD_RATE;

  // Turn on USART3 - USART Enable | Transmit Enable | Receive Enable
  USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

  // Set up an IRQ handler for USART3
  USART3->CR1 |= USART_CR1_RXNEIE;          // Enable the USART3 interrupt
  HAL_NVIC_EnableIRQ(USART3_4_IRQn);        // Turn on the IRQ in the NVIC
  HAL_NVIC_SetPriority(USART3_4_IRQn,1,2);  // Set the Interrupt's priority

  // Send the ascii code for the letter "R"
  char character = 'X';
 
  // Messages
  char clearterminal[] = "\e[H\e[2J";
  char clearline[] = "\eU";
  char notoptionstr[] = "\r\n\nThat is not an option.";
  char selectoption[] = "\r\n\nSelect: r = red \
                           \r\n        o = orange \
                           \r\n        g = green \
                           \r\n        b = blue\r\n\n\0";
    
  char selectmultioption[] = "\r\n\nSelect a color and then an LED condition: \
                                \n \
                                \r\n    r = red           0 = off \
                                \r\n    o = orange        1 = on \
                                \r\n    g = green         2 = toggle \
                                \r\n    b = blue\r\n\n\0";
  
  // // code for previous section that sends string
  SendString(clearterminal);
  SendString(selectoption);

  // delcare variables to be used in loop
  // char key2 = 'x';
  char key[] = "xx";
  bool badinput = false;
  uint8_t charcount = 0;
  uint32_t pinNum;
  
  while (1)
  {

    // // code for previous lab sections
    // // that sends a character and sends a string
    // SendChar(character);
    // HAL_Delay(1000);
    // My_HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

    // if (USART3->ISR & USART_ISR_RXNE){
    // key = USART3->RDR; // Read data
    
    while (charcount < 2){

      if (newdata == true){

        key[charcount] = received;  // grab data from from USART3
        newdata = false;            // reset flagqa
        SendChar(key[charcount]);   // echo character back
        charcount++;

      }
    }

    // two characters received, start count over
    charcount = 0;

    // if (newdata == true){
    //   key2 = received;
    //   newdata = false;

    // set the GPIO pin number
    switch (key[0]) {
      case 'r':
      case 'R':
        pinNum = GPIO_PIN_6;
        break;
      case 'g':
      case 'G':
        pinNum = GPIO_PIN_9;
        break;
      case 'b':
      case 'B':
        pinNum = GPIO_PIN_7;
        break;
      case 'o':
      case 'O':
        pinNum = GPIO_PIN_8;
        break;
      default:
        badinput = true;

    }

    // set the GPIO pin number
    switch (key[1]) {
      case '0':
        My_HAL_GPIO_WritePin(GPIOC, pinNum, RESET);
        break;
      case '1':
        My_HAL_GPIO_WritePin(GPIOC, pinNum, SET);
        break;
      case '2':
        My_HAL_GPIO_TogglePin(GPIOC, pinNum);
        break;
      default:
      badinput = true;
    }

    // if a bad value was input, send error message
    if (badinput == true){

        SendString(clearterminal);
        SendString(notoptionstr);
        // SendString(selectoption);
        SendString(selectmultioption);
        badinput = false;  // reset badletter flag
    }
  }
  // return -1;
}

void SendString(char str[]){

  // Do this until we hit End--of-String" character '\0'
  while (*str != '\0') {
    // If USART3 TX register is not empty
    while (USART3->ISR & USART_ISR_TXE){
      // Send character and increment pointer
      SendChar(*str++);
    }
  }
}


void SendChar(char send_char){

  if (USART3->ISR & USART_ISR_TXE){
    USART3->TDR = send_char;
  }
 
}


void USART3_4_IRQHandler(void){

  if (USART3->ISR & USART_ISR_RXNE){
      received = USART3->RDR; // Read data
      newdata = true;
      // Process received byte (e.g., store in buffer)
  }
  // if (USART3->ISR & USART_ISR_TXE){
  //     // TXE flag is set when data register is empty
  //     // You can now write new data to DR
  //     // Example: USART3->DR = next_byte;
  // }

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
