/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "stdio.h"
#include "string.h"

extern TIM_HandleTypeDef htim1;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VLC_TX_PORT GPIOA
#define VLC_TX_PIN  GPIO_PIN_8

#define BIT_PERIOD_US 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t Temp = 0, Rh = 0;
char msg[32];

uint8_t Live_TX_Bits[8]; // Stores the 8 bits of the current character being sent
char Current_Char;       // Shows which character is currently being bit-banged
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us (uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

uint8_t DHT11_Read (void) {
    uint8_t i, j;
    for (j=0; j<8; j++) {
        //Wait for the pin to go High
        while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)));

        // Measure how long the pin stays High
        __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset timer
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));   // Wait for pin to go Low
        uint16_t timer_val = __HAL_TIM_GET_COUNTER(&htim1); // Capture time

        // Logic: DHT11 '0' is 26us high, '1' is 70us high.
        // Decision threshold = 45us
        if (timer_val > 45) {
            i |= (1 << (7 - j));  // It's a 1
        } else {
            i &= ~(1 << (7 - j)); // It's a 0
        }
    }
    return i;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_Delay(1000); // Give sensor and OLED 1 full second to power up
  ssd1306_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t data[5] = {0};

	      // START SIGNAL
	      GPIO_InitTypeDef GPIO_InitStruct = {0};
	      GPIO_InitStruct.Pin = GPIO_PIN_1;
	      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	      GPIO_InitStruct.Pull = GPIO_NOPULL;
	      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
	      HAL_Delay(18); // 18ms Low
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	      delay_us(30);  // 30us High

	      // SWITCH TO INPUT
	      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	      GPIO_InitStruct.Pull = GPIO_PULLUP;
	      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	      // HANDSHAKE
	      // Wait for sensor to pull low (80us) and then high (80us)
	      uint32_t timeout = 0;
	      while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) && timeout < 1000) timeout++;
	      if(timeout < 1000) {
	          timeout = 0;
	          while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) && timeout < 1000) timeout++;
	          if(timeout < 1000) {
	              while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) && timeout < 1000) timeout++;

	              // --- READ DATA ---
	              data[0] = DHT11_Read(); // Humidity
	              data[1] = DHT11_Read();
	              data[2] = DHT11_Read(); // Temperature
	              data[3] = DHT11_Read();
	              data[4] = DHT11_Read(); // Checksum

	              // VALIDATE AND UPDATE
	              if (data[4] == (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
	                  Rh = data[0];
	                  Temp = data[2];
	              }
	          }
	      }

	      // DISPLAY & TRANSMIT
	      ssd1306_Fill(Black);
	      ssd1306_SetCursor(0, 0);
	      ssd1306_WriteString("REAL-TIME VLC", Font_7x10, White);

	      sprintf(msg, "TEMP: %d C", Temp);
	      ssd1306_SetCursor(10, 20);
	      ssd1306_WriteString(msg, Font_11x18, White);

	      sprintf(msg, "HUMI: %d %%", Rh);
	      ssd1306_SetCursor(10, 45);
	      ssd1306_WriteString(msg, Font_11x18, White);
	      ssd1306_UpdateScreen();

	      // Laser Modulation
	      // BULLETPROOF PPM TRANSMISSION
	      // START HEADER (60ms High)
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	      HAL_Delay(60);
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	      HAL_Delay(20); // Gap

	      // SEND HUMIDITY
	      for(int b=7; b>=0; b--){
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	          if((Rh >> b) & 1) HAL_Delay(15); // '1' is 15ms
	          else HAL_Delay(5);              // '0' is 5ms
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	          HAL_Delay(10); // Gap between bits
	      }

	      // SEND TEMPERATURE
	      for(int b=7; b>=0; b--){
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	          if((Temp >> b) & 1) HAL_Delay(15);
	          else HAL_Delay(5);
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	          HAL_Delay(10);
	      }

	      HAL_Delay(1500); // Wait before next update
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
	// CHANGE THIS: 16MHz / 16 = 1MHz (1 tick per microsecond)
	htim1.Init.Prescaler = 16-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xFFFF;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
	  Error_Handler();
	}
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT11_PIN_Pin|LASER_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DHT11_PIN_Pin LASER_OUT_Pin */
  GPIO_InitStruct.Pin = DHT11_PIN_Pin|LASER_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
