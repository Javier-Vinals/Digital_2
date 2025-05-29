/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
#define TXBUFFERSIZE 4
#define RXBUFFERSIZE 8

int count = 0;

uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

uint8_t valor_aTx0;
uint8_t valor_aTx1;
uint8_t valor_aTx2;
uint8_t valor_aTx3;

uint8_t Dato[2];

uint8_t valorLDR1;
uint8_t valorLDR2;
uint8_t valorLDR3;
uint8_t valorLDR4;

uint8_t sumaDeUnos = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  valorLDR1 = HAL_GPIO_ReadPin(GPIOB, LDR_1_Pin);
	  	 	  if (valorLDR1)
	  	 	  {
	  	 		  HAL_GPIO_WritePin(GPIOA, LED_G_1_Pin, GPIO_PIN_RESET);
	  	 	      HAL_GPIO_WritePin(GPIOA, LED_R_1_Pin, GPIO_PIN_SET);
	  	 	      valor_aTx0 = '1';
	  	 	  }
	  	 	  else
	  	 	  {
	  	 		  HAL_GPIO_WritePin(GPIOA,  LED_G_1_Pin, GPIO_PIN_SET);
	  	 	      HAL_GPIO_WritePin(GPIOA,  LED_R_1_Pin, GPIO_PIN_RESET);
	  	 	      valor_aTx0 = '0';
	  	 	  }


	  valorLDR2 = HAL_GPIO_ReadPin(GPIOB, LDR_2_Pin);
	  	 	 if (valorLDR2)
	  	 	 {
	  	 		 HAL_GPIO_WritePin(GPIOB, LED_G_2_Pin, GPIO_PIN_RESET);
	  	 		 HAL_GPIO_WritePin(GPIOC, LED_R_2_Pin, GPIO_PIN_SET);
	  	 		valor_aTx1 = '1';
	  	 	 }
	  	 	 else
	  	 	 {
	  	 		 HAL_GPIO_WritePin(GPIOB,  LED_G_2_Pin, GPIO_PIN_SET);
	  	 		 HAL_GPIO_WritePin(GPIOC,  LED_R_2_Pin, GPIO_PIN_RESET);
	  	 		valor_aTx1 = '0';
	  	 	 }


	  valorLDR3 = HAL_GPIO_ReadPin(GPIOB, LDR_3_Pin);
	  	 	if (valorLDR3)
	  	 	{
	  	 		HAL_GPIO_WritePin(GPIOA, LED_G_3_Pin, GPIO_PIN_RESET);
	  	 		HAL_GPIO_WritePin(GPIOA, LED_R_3_Pin, GPIO_PIN_SET);
	  	 	}
	  	 	else
	  	 	{
	  	 		HAL_GPIO_WritePin(GPIOA,  LED_G_3_Pin, GPIO_PIN_SET);
	  	 		HAL_GPIO_WritePin(GPIOA,  LED_R_3_Pin, GPIO_PIN_RESET);
	  	 	}

	  valorLDR4 = HAL_GPIO_ReadPin(GPIOB, LDR_4_Pin);
	  	 	if (valorLDR4)
	  	 	{
	  	 		HAL_GPIO_WritePin(GPIOC, LED_G_4_Pin, GPIO_PIN_RESET);
	  	 		HAL_GPIO_WritePin(GPIOA, LED_R_4_Pin, GPIO_PIN_SET);
	  	 	}
	  	 	else
	  	 	{
	  	 		HAL_GPIO_WritePin(GPIOC,  LED_G_4_Pin, GPIO_PIN_SET);
	  	 		HAL_GPIO_WritePin(GPIOA,  LED_R_4_Pin, GPIO_PIN_RESET);
	  	 	}

	  sumaDeUnos = 0;
	  for (int i = 0; i < RXBUFFERSIZE; i++) {
		  if (aRxBuffer[i] == 1) {
			  sumaDeUnos++;
		  }}

	  if(sumaDeUnos == 0){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_RESET);

	  }else if(sumaDeUnos == 1){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_RESET);

	  }else if(sumaDeUnos == 2){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }else if(sumaDeUnos == 3){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }else if(sumaDeUnos == 4){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }else if(sumaDeUnos == 5){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }else if(sumaDeUnos == 6){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }else if(sumaDeUnos == 7){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_RESET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_RESET);

	  }else if(sumaDeUnos == 8){
		  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOA, SEG_C_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_E_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_F_Pin, GPIO_PIN_SET);
		  		  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin, GPIO_PIN_SET);

	  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 170;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_B_Pin|SEG_A_Pin|LED_G_3_Pin|LED_R_3_Pin
                          |LED_R_1_Pin|LED_G_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_R_4_Pin|SEG_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_G_Pin|SEG_D_Pin|SEG_F_Pin|SEG_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_2_GPIO_Port, LED_R_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_4_GPIO_Port, LED_G_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_2_GPIO_Port, LED_G_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_B_Pin SEG_A_Pin LED_R_4_Pin LED_G_3_Pin
                           LED_R_3_Pin LED_R_1_Pin LED_G_1_Pin SEG_C_Pin */
  GPIO_InitStruct.Pin = SEG_B_Pin|SEG_A_Pin|LED_R_4_Pin|LED_G_3_Pin
                          |LED_R_3_Pin|LED_R_1_Pin|LED_G_1_Pin|SEG_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_G_Pin SEG_D_Pin SEG_F_Pin SEG_E_Pin
                           LED_G_2_Pin */
  GPIO_InitStruct.Pin = SEG_G_Pin|SEG_D_Pin|SEG_F_Pin|SEG_E_Pin
                          |LED_G_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LDR_4_Pin LDR_1_Pin LDR_2_Pin LDR_3_Pin */
  GPIO_InitStruct.Pin = LDR_4_Pin|LDR_1_Pin|LDR_2_Pin|LDR_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_2_Pin LED_G_4_Pin */
  GPIO_InitStruct.Pin = LED_R_2_Pin|LED_G_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(hi2c);
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

	    aTxBuffer[0] = valor_aTx0;
	    aTxBuffer[1] = valor_aTx1;
	    aTxBuffer[2] = valor_aTx2;
	    aTxBuffer[3] = valor_aTx3;

}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
    // Prueba si realmente entra al callback
 //   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // LED verde



}


void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
 //   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // LED ON permanente en error
}



void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
	if (TransferDirection == I2C_DIRECTION_TRANSMIT){
		if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t*) Dato,
				RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) !=HAL_OK) {
			Error_Handler();
		}
	} else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
		if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*) aTxBuffer, 4, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
			Error_Handler();
		}
	}

	uint8_t valor = Dato[0];

	    for (int i = 0; i < 8; i++) {
	        aRxBuffer[i] = (valor >> (7 - i)) & 0x01;}
}


//void HAL_I2C_ErrorCallback(I2C_HandlerTypeDef *I2cHandle)
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
