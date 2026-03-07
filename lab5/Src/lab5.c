#include "main.h"
#include "stm32f0xx_hal.h"
#include "hal_gpio.h"
#include "stm32f0xx_hal_gpio.h"

void SystemClock_Config(void);
void Error_Handler(void);
void WriteI2C(uint32_t slaveAddress, uint32_t writeAddress, uint32_t nbytes, uint8_t data);
uint8_t ReadI2C(uint32_t deviceAddress, uint32_t memAddress, uint32_t nbytes);
int twosCompToDec(uint32_t twosCompNuml);
uint32_t readbyte = 0x0000;

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
  // Enable the I2C2 peripheral in the RCC
  // RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
  __HAL_RCC_I2C2_CLK_ENABLE();


  // Configure LED GPIOC Output pins for LED use
  // Set up pins connected to LEDs as Ouput w/out Pull-Up/Pull-Down
  GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, // Pin   - GPIOx_MODER
                              GPIO_MODE_OUTPUT_PP,      // Mode  - GPIOx_OTYPER
                              GPIO_NOPULL,              // Pull  - GPIOx_PUPDR
                              GPIO_SPEED_FREQ_LOW};    // Speed - GPIOx_OSPEEDR
  My_HAL_GPIO_Init(GPIOC, &initStr); // Initializes pins PC8 & PC9
  // My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET); // turn on green LED

  /********************************************
  // Set up the gyroscope chip for use with I2C2
  // instead of default SPI mode
  ********************************************/

  // Configure PB11
  initStr.Pin = GPIO_PIN_11,
  initStr.Mode = GPIO_MODE_AF_OD,
  initStr.Pull = GPIO_NOPULL; // GPIO_PULLUP;
  // initStr.Pull = GPIO_NOPULL,
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
    // Initializes PB11
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set PortB Pin11 to Alternate Function 1 (AF1)
  // which connects it to the STM32 I2C_SDA pin
  // See page 162 & 163/1017 in periph. ref. manual
  // AF1 = (0x01), pin 11 = postion 3 in high register [1]
  GPIOB->AFR[1] |= (0x01 << GPIO_AFRH_AFRH3_Pos);


  // Set GPIOB Pins 13 alternate function mode
  initStr.Pin = GPIO_PIN_13;
  initStr.Mode = GPIO_MODE_AF_OD;
  initStr.Pull = GPIO_NOPULL; // PULLUP;
  // initStr.Pull = GPIO_NOPULL;
  initStr.Speed = GPIO_SPEED_FREQ_LOW;
  // Initializes pin PB13
  HAL_GPIO_Init(GPIOB, &initStr);

  // Set Port B pin 13 to 
  // which connects it to the STM32 I2C_SCL pin
  // AF1 = (0x05), pin 13 = postion 5 in high register [1]
  GPIOB->AFR[1] |= (0x05 << GPIO_AFRH_AFRH5_Pos);

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

  // Turn of / Reset I2C2 in the I2C2_CR1 register prior to 
  // configuring the register by setting PE (bit0) to 0
  I2C2->CR1 &= ~I2C_CR1_PE_Msk;
  HAL_Delay(10);  // delay to let the reset take hold

  // Set the timing register TIMINGR to use100kHz (standard-mode I2C)
  // See pages 666 & 691/1017 in periph. ref. manual
  // Reset value: 0x0000 0000
  // 8MHz values:  PRESC = 0x1 | SCLL = 0x13 | SCLH = 0xF | SDADEL = 0x2 | SCLDEL = 0x4 
  I2C2->TIMINGR = (0x1 << 28   | 0x13 << 0   | 0xF << 8   | 0x2 << 16    | 0x4 << 20   );

  // Enable I2C2 in the I2C2_CR1 register by setting PE (bit0) to 1
  I2C2->CR1 |= (1 << I2C_CR1_PE_Pos);

  /********************************************
  // Read the WHO_AM_I register on the gyroscope I2C peripheral
  // Turn on green LED of correct values is read, or RED if wrong
  ********************************************/

  uint32_t gyroAddress = 0x69;
  uint32_t WHO_AM_I = 0x0F;
  uint32_t byteread;

  // read a byte from slave device memory address
  // and put it into the global variable "readbyte"
  byteread = ReadI2C(gyroAddress, WHO_AM_I, 1);

    // Set the stop bit in the CR2 register to release the I2C2 bus
  I2C2->CR2 |= I2C_CR2_STOP_Msk; 

  // Check the contents of the RXDR register to see
  //  if it matches the expected value of 0xD3
  if(byteread == 0xD3){
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);   // green
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET); // red
  }  else{
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);   // red
    My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET); // green
  }

  // the gyroscopes control register 1
  uint32_t gyroCR1 = 0x20;

  // Configure the gyroscope so we can read the X an dY axes.
  // 0xB = 0000 1011 which enables X and Y, and puts the gyro in sleep/normal mode
  WriteI2C(gyroAddress, gyroCR1, 1, 0xB);

  // // debug code to see if gyroCR1 was written
  // byteread = ReadI2C(gyroAddress, gyroCR1, 1);
  // byteread = ReadI2C(gyroAddress, WHO_AM_I, 1);

  // X and Y register high and low addresses on the gyroscope chip
  uint8_t OUT_X_LAddr = 0x28;
  uint8_t OUT_X_HAddr = 0x29;
  uint8_t OUT_Y_LAddr = 0x2A;
  uint8_t OUT_Y_HAddr = 0x2B;
  
  
  // variables used to read the X and Y registers
  uint8_t  OUT_X_H;
  uint8_t  OUT_X_L;
  uint8_t  OUT_Y_H;
  uint8_t  OUT_Y_L;
  int16_t OUT_X;
  int16_t OUT_Y;
  int16_t Hysterisis = 1000;

 while (1) 
  {

    OUT_X_L = ReadI2C(gyroAddress, OUT_X_LAddr, 1);
    OUT_X_H = ReadI2C(gyroAddress, OUT_X_HAddr, 1);
    OUT_Y_L = ReadI2C(gyroAddress, OUT_Y_LAddr, 1);
    OUT_Y_H = ReadI2C(gyroAddress, OUT_Y_HAddr, 1);

    OUT_X = twosCompToDec(((OUT_X_H << 8) | (OUT_X_L)));
    OUT_Y = twosCompToDec(((OUT_Y_H << 8) | (OUT_Y_L)));

    // This is a breakdown of which LED is connected to which Port C pin
    // LD3 (top)    red    user LED is connected to the I/O PC6 of the STM32F072RBT6
    // LD4 (left)   orange user LED is connected to the I/O PC8 of the STM32F072RBT6
    // LD5 (right)  green  user LED is connected to the I/O PC9 of the STM32F072RBT6
    // LD6 (bottom) blue   user LED is connected to the I/O PC7 of the STM32F072RBT6

    // Turn on/off green and orange LEDs according to angular acceleration direction.
    if (OUT_X >= 0 + Hysterisis){
      // turn the green LED off and the orange LED on
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
    } else if (OUT_X <= -Hysterisis){
      // turn the green LED on and the orange LED off
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, SET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);  
    } else {
      // turn the green LED off and the ornage LED off
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, RESET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);
    }

    // Turn on/off red and blue LEDs according to angular acceleration direction.
    if (OUT_Y >=0 + Hysterisis){ // X angle is a Positive number
      // turn the red LED off and the blue LED on
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
    } else if (OUT_Y <= -Hysterisis){
      // turn the red LED on and the blue LED off
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, SET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
    } else {
      // turn the red LED off and the blue LED off
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, RESET);
      My_HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
    }

    HAL_Delay(100);  // delay 100ms 
 
  }
  return -1;
}

int twosCompToDec(uint32_t twosCompNuml){
    if ((twosCompNuml & 0x80000000) == 0) {
        return twosCompNuml; // Positive number
    } else {
        return -(~twosCompNuml + 1); // Negative number
    }
}

uint8_t ReadI2C(uint32_t deviceAddress, uint32_t memAddress, uint32_t nbytes){

  // Wait for BUSY flag to clear
  while (I2C2->ISR & I2C_ISR_BUSY);

  // Send the device the memeory address that you
  // want it to read data from and send back to you

  // Put the 7-bit I2C address of the slave device 
  // into the I2C2_CR2 SADD bits [7:1] - NOT including bit 0
  I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
  I2C2->CR2 |= (deviceAddress << 1);

  I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;           // clear NBYTES
  I2C2->CR2 |= (1 << I2C_CR2_NBYTES_Pos);     // number of bytes to write
  I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;           // clear RD_WRN
  I2C2->CR2 |= (0 << I2C_CR2_RD_WRN_Pos);     // (0 << 10) 0 for write
  I2C2->CR2 |= (1 << I2C_CR2_START_Pos);      // set Start bit

  // wait until either of the TXIS or NACKF flags are set
  while((I2C2->ISR & I2C_ISR_TXIS) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0);

  // TXIS flag is set, continue
  if((I2C2->ISR & I2C_ISR_TXIS_Msk)){
   // Write to slave device telling it which address read data from
    I2C2->TXDR = memAddress;
  } else if (I2C2->ISR & I2C_ISR_NACKF_Msk){ // If NACKF .. throw error
      Error_Handler();  // This disables interrupts and enters an infinite loop
  }

  // Wait until the TC (Transfer Complete) flag is set
  while(!(I2C2->ISR & I2C_ISR_TC));

  I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;             // clear NBYTES
  I2C2->CR2 |= (nbytes << I2C_CR2_NBYTES_Pos);  // (0x1 << 16)
  I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;             // clear RD_WRN
  I2C2->CR2 |= (1 << I2C_CR2_RD_WRN_Pos);       // (1 << 10) 1 for read
  I2C2->CR2 |= (1 << I2C_CR2_START_Pos);        // set Start bit

  // wait until either the RXNE or NACKF flags are set
  while((I2C2->ISR & I2C_ISR_RXNE) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0);

  // TXIS flag is set, continue
  if((I2C2->ISR & I2C_ISR_RXNE_Msk)){
    // get received byte
    readbyte = I2C2->RXDR;
  } else if (I2C2->ISR & I2C_ISR_NACKF_Msk){ // If NACKF ..set throw error
    Error_Handler();  // This disables interrupts and enters an infinite loop
  }

  // Wait until the TC (Transfer Complete) flag is set
  while(!(I2C2->ISR & I2C_ISR_TC_Msk)){
    // Do nothing, just wait
  }

  I2C2->CR2 |= (1 << I2C_CR2_STOP_Pos);       // set Stop bit

  return readbyte;

}

void WriteI2C(uint32_t slaveAddress, uint32_t writeAddress, uint32_t nbytes, uint8_t data){

  // Wait for BUSY flag to clear
  while (I2C2->ISR & I2C_ISR_BUSY);

  // Write the 7-bit I2C address of the slave device 
  // into the I2C2_CR2 SADD bits [7:1] - NOT including bit 0
  I2C2->CR2 &= ~I2C_CR2_SADD_Msk;
  I2C2->CR2 |= (slaveAddress << 1);

  I2C2->CR2 &= ~I2C_CR2_NBYTES_Msk;         // clear NBYTES
  I2C2->CR2 |= ((nbytes+1) << I2C_CR2_NBYTES_Pos);   // 1 writeAddress to send
  I2C2->CR2 &= ~I2C_CR2_RD_WRN_Msk;         // clear RD_WRN
  I2C2->CR2 |= (0 << I2C_CR2_RD_WRN_Pos);   // (0 << 10) 0 for write
  I2C2->CR2 |= (1 << I2C_CR2_START_Pos);    // set Start bit

  // wait until either of the TXIS or NACKF flags are set
  while((I2C2->ISR & I2C_ISR_TXIS) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0);

  // TXIS flag is set, continue
  if((I2C2->ISR & I2C_ISR_TXIS_Msk)){
   // Write to slave device telling it which address read data from
    I2C2->TXDR = writeAddress;
  } else if (I2C2->ISR & I2C_ISR_NACKF_Msk){ // If NACKF .. throw error
      Error_Handler();  // This disables interrupts and enters an infinite loop
  }

  // wait until either of the TXIS or NACKF flags are set
  while((I2C2->ISR & I2C_ISR_TXIS) == 0 && (I2C2->ISR & I2C_ISR_NACKF) == 0);

  // TXIS flag is set, continue
  if((I2C2->ISR & I2C_ISR_TXIS_Msk)){
    // get received byte
    I2C2->TXDR = data;
  } else if (I2C2->ISR & I2C_ISR_NACKF_Msk){ // If NACKF ..set throw error
    Error_Handler();  // This disables interrupts and enters an infinite loop
  }

  // Wait until the TC (Transfer Complete) flag is set
  while(!(I2C2->ISR & I2C_ISR_TC_Msk)){}; // Do nothing, just wait

  I2C2->CR2 |= (1 << I2C_CR2_STOP_Pos);       // set Stop bit

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