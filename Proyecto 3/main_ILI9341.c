/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "ili9341.h"
#include "Bitmaps.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

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

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t bit0 = 0;
uint8_t bit1 = 0;
uint8_t bit2 = 0;
uint8_t bit3 = 0;
uint8_t bit4 = 0;
uint8_t bit5 = 0;
uint8_t bit6 = 0;
uint8_t bit7 = 0;

uint8_t bitVars[8] = {0};     // Variables encendidas/apagadas según los bits
uint8_t bitFlags[8] = {0};    // Estados previos de los bits
uint8_t bitCounter = 0;       // Contador de bits en 1

uint8_t valor_aTx5;
uint8_t valor_aTx6;
uint8_t valor_aTx7;
uint8_t valor_aTx8;

uint8_t valorLDR5;
uint8_t valorLDR6;
uint8_t valorLDR7;
uint8_t valorLDR8;

#define TXBUFFERSIZE 4
#define RXBUFFERSIZE 8

int count = 0;

uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];

uint8_t Dato[2];


uint8_t contadorseg7 = 1;
uint8_t contador7segpos = 1;
uint8_t contadorparqueo = 0;
uint8_t i2c_rx_buffer[1];
uint16_t ldr_values[4];  // Guardará las lecturas de los LDR
uint8_t i2c_tx_buffer[8]; // Para enviar 4 valores de 2 bytes cada uno

uint8_t rx_data;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DelaySoftware(uint32_t delay) {
    for (volatile uint32_t i = 0; i < delay * 8000; i++); // Aproximadamente 1 ms si el MCU corre a 72 MHz
}

void DibujarVehiculos(uint8_t cantidad) {

    for (uint8_t i = 0; i < 8; i++) {
        uint8_t row = i / 4;

        if (i < cantidad) {
            // Dibuja carrito en su lugar
            LCD_Sprite(1, 1, 36, 35, carritomimido, 4, i % 4, row, 0);
        } else {
            // "Borra" carrito con uno invisible: dibuja fuera de pantalla
            LCD_Sprite(240, 320, 36, 35, carritomimido, 4, i % 4, row, 0);
        }
    }
}



void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        uint8_t valor = i2c_rx_buffer[0];

        if (valor >= '0' && valor <= '8') {
            contadorparqueo = valor;  // ASCII '0'-'8'
            contadorseg7 = 1;
        }

        HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buffer, 1);
    }
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_Clear(0x00);

	// Graficacion basica

	LCD_Print("Parqueos", 230, 80, 1, 0xffc0, 0);
	LCD_Print("disponibles", 220, 95, 1, 0xffc0, 0);
	LCD_Sprite(240,120,41,52,seg7,9,0,0,0);
	LCD_Sprite(2,2,168,116,parqueotop,1,0,0,0);
	LCD_Sprite(2,117,168,116,parqueobotom,1,0,0,0);

	//Activacion del protocolo de comunicacion i2c

    if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
    	  Error_Handler();
      }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		//Mitad izquierda
		//Deteccion de parqueo individual y graficacion para cada slot
		if (bitVars[0]==1){
			LCD_Sprite(28,76,27,26,carritomimido,2,0,0,0);
			LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
		}else{
			LCD_Sprite(28,76,27,26,carritomimido,2,1,0,0);
			LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
		}
		if (bitVars[1]==1){
					LCD_Sprite(28,106,27,26,carritomimido,2,0,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}else{
					LCD_Sprite(28,106,27,26,carritomimido,2,1,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}

		if (bitVars[2]==1){
					LCD_Sprite(28,136,27,26,carritomimido,2,0,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}else{
					LCD_Sprite(28,136,27,26,carritomimido,2,1,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}

		if (bitVars[3]==1){
					LCD_Sprite(28,166,27,26,carritomimido,2,0,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}else{
					LCD_Sprite(28,166,27,26,carritomimido,2,1,0,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}
		//Mitad derecha
		if (bitVars[4]==1){
					LCD_Sprite(120,76,27,26,carritomimido,2,0,1,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}else{
					LCD_Sprite(120,76,27,26,carritomimido,2,1,1,0);
					LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
				}
		if (bitVars[5]==1){
							LCD_Sprite(120,106,27,26,carritomimido,2,0,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}else{
							LCD_Sprite(120,106,27,26,carritomimido,2,1,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}

		if (bitVars[6]==1){
							LCD_Sprite(120,136,27,26,carritomimido,2,0,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}else{
							LCD_Sprite(120,136,27,26,carritomimido,2,1,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}

		if (bitVars[7]==1){
							LCD_Sprite(120,166,27,26,carritomimido,2,0,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}else{
							LCD_Sprite(120,166,27,26,carritomimido,2,1,1,0);
							LCD_Sprite(240,120,41,52,seg7,9,bitCounter,0,0);
						}

		//Seccion de lectura de las resistencias LDR
		//Dividido en cada pin, aqui se lee individualmente
		valorLDR5 = HAL_GPIO_ReadPin(GPIOC, LDR_5_Pin);
				  	 	  if (valorLDR5)
				  	 	  {
				  	 		  HAL_GPIO_WritePin(GPIOB, LED_G_5_Pin, GPIO_PIN_RESET);
				  	 	      HAL_GPIO_WritePin(GPIOC, LED_R_5_Pin, GPIO_PIN_SET);
				  	 	      valor_aTx5 = '1';
				  	 	  }
				  	 	  else
				  	 	  {
				  	 		  HAL_GPIO_WritePin(GPIOB,  LED_G_5_Pin, GPIO_PIN_SET);
				  	 	      HAL_GPIO_WritePin(GPIOC,  LED_R_5_Pin, GPIO_PIN_RESET);
				  	 	      valor_aTx5 = '0';
				  	 	  }


				  	 	valorLDR6 = HAL_GPIO_ReadPin(GPIOC, LDR_6_Pin);
				  	 					  	 	  if (valorLDR6)
				  	 					  	 	  {
				  	 					  	 		  HAL_GPIO_WritePin(GPIOA, LED_G_6_Pin, GPIO_PIN_RESET);
				  	 					  	 	      HAL_GPIO_WritePin(GPIOA, LED_R_6_Pin, GPIO_PIN_SET);
				  	 					  	 	      valor_aTx6 = '1';
				  	 					  	 	  }
				  	 					  	 	  else
				  	 					  	 	  {
				  	 					  	 		  HAL_GPIO_WritePin(GPIOA,  LED_G_6_Pin, GPIO_PIN_SET);
				  	 					  	 	      HAL_GPIO_WritePin(GPIOA,  LED_R_6_Pin, GPIO_PIN_RESET);
				  	 					  	 	      valor_aTx6 = '0';
				  	 					  	 	  }

				  	 					  	 valorLDR7 = HAL_GPIO_ReadPin(GPIOC, LDR_7_Pin);
				  	 					  	 				  	 	  if (valorLDR7)
				  	 					  	 				  	 	  {
				  	 					  	 				  	 		  HAL_GPIO_WritePin(GPIOC, LED_G_7_Pin, GPIO_PIN_RESET);
				  	 					  	 				  	 	      HAL_GPIO_WritePin(GPIOA, LED_R_7_Pin, GPIO_PIN_SET);
				  	 					  	 				  	 	      valor_aTx7 = '1';
				  	 					  	 				  	 	  }
				  	 					  	 				  	 	  else
				  	 					  	 				  	 	  {
				  	 					  	 				  	 		  HAL_GPIO_WritePin(GPIOC,  LED_G_7_Pin, GPIO_PIN_SET);
				  	 					  	 				  	 	      HAL_GPIO_WritePin(GPIOA,  LED_R_7_Pin, GPIO_PIN_RESET);
				  	 					  	 				  	 	      valor_aTx7 = '0';
				  	 					  	 				  	 	  }

				  	 					  	 				  	valorLDR8 = HAL_GPIO_ReadPin(GPIOC, LDR_8_Pin);
				  	 					  	 				  					  	 	  if (valorLDR8)
				  	 					  	 				  					  	 	  {
				  	 					  	 				  					  	 		  HAL_GPIO_WritePin(GPIOC, LED_G_8_Pin, GPIO_PIN_RESET);
				  	 					  	 				  					  	 	      HAL_GPIO_WritePin(GPIOC, LED_R_8_Pin, GPIO_PIN_SET);
				  	 					  	 				  					  	 	      valor_aTx5 = '1';
				  	 					  	 				  					  	 	  }
				  	 					  	 				  					  	 	  else
				  	 					  	 				  					  	 	  {
				  	 					  	 				  					  	 		  HAL_GPIO_WritePin(GPIOC,  LED_G_8_Pin, GPIO_PIN_SET);
				  	 					  	 				  					  	 	      HAL_GPIO_WritePin(GPIOC,  LED_R_8_Pin, GPIO_PIN_RESET);
				  	 					  	 				  					  	 	      valor_aTx5 = '0';
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hi2c1.Init.OwnAddress1 = 138;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, LED_R_5_Pin|LED_R_8_Pin|LED_G_8_Pin|LED_G_7_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_R_7_Pin|LED_G_6_Pin|LED_R_6_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_5_GPIO_Port, LED_G_5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_R_5_Pin LED_R_8_Pin LED_G_8_Pin LED_G_7_Pin */
  GPIO_InitStruct.Pin = LED_R_5_Pin|LED_R_8_Pin|LED_G_8_Pin|LED_G_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LDR_8_Pin LDR_7_Pin LDR_6_Pin LDR_5_Pin */
  GPIO_InitStruct.Pin = LDR_8_Pin|LDR_7_Pin|LDR_6_Pin|LDR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin SD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_7_Pin LED_G_6_Pin LED_R_6_Pin */
  GPIO_InitStruct.Pin = LED_R_7_Pin|LED_G_6_Pin|LED_R_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_G_5_Pin */
  GPIO_InitStruct.Pin = LED_G_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_G_5_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
	HAL_I2C_EnableListen_IT(hi2c);
}


void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

		//Definir cada pin a un valor de la cadena.
	    aTxBuffer[0] = valor_aTx5;
	    aTxBuffer[1] = valor_aTx6;
	    aTxBuffer[2] = valor_aTx7;
	    aTxBuffer[3] = valor_aTx8;

}




void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
//    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // LED ON permanente en error
}



void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode){
    if (TransferDirection == I2C_DIRECTION_TRANSMIT){
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t*) Dato,
                RXBUFFERSIZE, I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
            Error_Handler();
        }
    } else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*) aTxBuffer, 4, I2C_FIRST_AND_LAST_FRAME) != HAL_OK){
            Error_Handler();
        }
    }

    uint8_t valor = Dato[0];

    for (int i = 0; i < 8; i++) {
        uint8_t bit = (valor >> (7 - i)) & 0x01;

        // Encender/apagar variables individuales
        bitVars[i] = bit;

        // Actualizar contador solo si cambia el estado
        if (bit == 1 && bitFlags[i] == 0) {
            bitFlags[i] = 1;
            bitCounter++;
        } else if (bit == 0 && bitFlags[i] == 1) {
            bitFlags[i] = 0;
            bitCounter--;
        }
    }
}
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
	while (1) {
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
