/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

//#define MASTER_BOARD
#define I2C_ADDRESS        0x3E
#define MASTER_REQ_READ    0x12
#define MASTER_REQ_WRITE   0x34

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);				//Use USART2
	return ch;
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);


/* Private user code ---------------------------------------------------------*/


uint8_t aTxBuffer[] = "I2C Master Transmit data to slave, 1234567890 abcdefghijklmnopqrstuvwxyz";
uint8_t aRxBuffer[sizeof(aTxBuffer)];

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();


  printf("HW init OK\r\n");
  printf("size TX: %d \r\n", sizeof(aTxBuffer));

  /* Infinite loop */
  while (1)
  {

#ifdef MASTER_SENT
    if(HAL_I2C_Master_Transmit_IT(&hi2c1, I2C_ADDRESS, (uint8_t*)&aTxBuffer, sizeof(aTxBuffer)) != HAL_OK){
    	printf("I2C Master TX fail\r\n");
    }
#else
    if(HAL_I2C_Master_Receive_IT(&hi2c1, I2C_ADDRESS, (uint8_t*)&aRxBuffer, sizeof(aRxBuffer)) != HAL_OK){
		printf("I2C Master RX fail\r\n");
	}
#endif

    HAL_Delay(1000);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType 		= RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState 			= RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 		= RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM 			= 16;
  RCC_OscInitStruct.PLL.PLLN 			= 336;
  RCC_OscInitStruct.PLL.PLLP 			= RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ 			= 2;
  RCC_OscInitStruct.PLL.PLLR 			= 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Instance 				= I2C1;
  hi2c1.Init.ClockSpeed 		= 400000;
  hi2c1.Init.DutyCycle 			= I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 		= 64;
  hi2c1.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 		= 0;
  hi2c1.Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance				= I2C2;
  hi2c2.Init.ClockSpeed 		= 100000;
  hi2c2.Init.DutyCycle 			= I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 		= I2C_ADDRESS;
  hi2c2.Init.AddressingMode 	= I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 		= 0;
  hi2c2.Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode 		= I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
	  Error_Handler();
  }

#ifdef MASTER_SENT
  if(HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*)&aRxBuffer, sizeof(aRxBuffer))!= HAL_OK){
	  printf("I2C Slave RX fail\r\n");
  }
#else
  if(HAL_I2C_Slave_Transmit_IT(&hi2c2, (uint8_t*)&aTxBuffer, sizeof(aTxBuffer))!= HAL_OK){
 	  printf("I2C Slave TX fail\r\n");
  }
#endif

}


void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}


void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}


void I2C2_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c2);
}


void I2C2_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c2);
}


/*************************************************************************
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note
  * @retval None
  ************************************************************************/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  if(I2cHandle->Instance == I2C1)
  {
	  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
	  {
		  printf("I2C1 error \r\n");
	  }
  }else if(I2cHandle->Instance == I2C2){
	  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
	  {
		  printf("I2C2 error \r\n");
	  }
  }
}


/*************************************************************************
  * @brief  I2C slave Tx  callbacks
  * @param  I2cHandle: I2C handle
  * @note
  * @retval None
  ************************************************************************/
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1){
		printf("Master transmit finish\r\n");
	}
	else if(hi2c->Instance == I2C2)
	{
		printf("Master RX: %s\r\n", aRxBuffer);
		memset(aRxBuffer, '\0',sizeof(aRxBuffer));

		if(HAL_I2C_Slave_Transmit_IT(&hi2c2, (uint8_t*)&aTxBuffer, sizeof(aTxBuffer))!= HAL_OK){
			printf("I2C Slave TX fail\r\n");
		}
	}
}


/*************************************************************************
  * @brief  I2C slave Tx  callbacks
  * @param  I2cHandle: I2C handle
  * @note
  * @retval None
  ************************************************************************/
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1){
		printf("Slave RX: %s\r\n", aRxBuffer);
		memset(aRxBuffer, '\0',sizeof(aRxBuffer));
	}
	else if(hi2c->Instance == I2C2)
	{
		printf("Slave RX: %s\r\n", aRxBuffer);
		memset(aRxBuffer, '\0',sizeof(aRxBuffer));

		if(HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*)&aRxBuffer, sizeof(aRxBuffer))!= HAL_OK){
			printf("I2C Slave RX fail\r\n");
		}
	}
}





//====================================================================================================================//



/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance 				= SPI2;
  hspi2.Init.Mode 				= SPI_MODE_SLAVE;
  hspi2.Init.Direction 			= SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize 			= SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity 		= SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase 			= SPI_PHASE_1EDGE;
  hspi2.Init.NSS 				= SPI_NSS_SOFT;
  hspi2.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode 			= SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation 	= SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial 		= 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}


/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance 				= USART2;
  huart2.Init.BaudRate 			= 115200;
  huart2.Init.WordLength 		= UART_WORDLENGTH_8B;
  huart2.Init.StopBits 			= UART_STOPBITS_1;
  huart2.Init.Parity 			= UART_PARITY_NONE;
  huart2.Init.Mode 				= UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
  huart2.Init.OverSampling 		= UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin 		= B1_Pin;
  GPIO_InitStruct.Mode 		= GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin 		= LD2_Pin;
  GPIO_InitStruct.Mode 		= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;
  GPIO_InitStruct.Speed 	= GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	printf("error handler\r\n");
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
