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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t PUNTOS_PARA_GANAR = 3;

extern uint8_t fondo[];
uint8_t startsequence =0;
uint8_t startsequence_2 =0;
uint8_t play;
uint8_t jugar = 0;
uint8_t movimiento_nav1;
uint16_t posoriginalx;
uint8_t posoriginaly_1;
uint8_t posoriginaly_2;
uint8_t posactualx;
uint8_t avanzar_1 = 0;
uint8_t atras_1 = 0;
uint8_t izquierda = 0;
uint8_t derecha = 0;
uint8_t disp_1 = 0;
uint8_t jugador_1 = 4;
uint8_t movimiento_juego = 0;
#define EXPLOSION_W 32
#define EXPLOSION_H 32
#define EXPLOSION_FRAMES 6

/*
typedef struct {
	uint16_t x;
	uint16_t y;
	uint32_t last_update;
	uint8_t direccion;
} Nave;*/

// Definición de estructura del jugador
typedef struct {
    int x, y;
    const uint8_t *sprite;
    int frame;
    int flip;
} Player;

typedef struct {
    int active;
    int x, y;
    int direction; // 1 para abajo, -1 para arriba
} Bullet;

Bullet bullet1 = {0, 0, 0, -1};
Bullet bullet2 = {0, 0, 0, 1};


uint8_t uart_rx; // Variable para recibir el comando

uint8_t rx_data;

Player player1 = {150, 220, nave_1, 0, 0};
Player player2 = {150, 10, nave_2, 0, 0};


// Tamaño del sprite
#define SPRITE_W 19
#define SPRITE_H 20

/*Nave nave1 = {160, 220, 0, 0};
Nave nave2 = {160, 20, 0, 0};  // Posición inicial de nave 2*/

uint32_t intervalo_movimiento = 50; // ms

uint8_t puntos_1 = 3;
uint8_t puntos_2 = 3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void animate_explosion(int x, int y) {
    for (int frame = 0; frame < EXPLOSION_FRAMES; frame++) {
        LCD_Sprite(x, y, EXPLOSION_W, EXPLOSION_H, explosion_sprite, EXPLOSION_FRAMES, frame, 0, 0);
        HAL_Delay(50);
    }
    FillRect(x, y, EXPLOSION_W, EXPLOSION_H, 0x0000); // Limpia después de la animación
}

void draw_player_sprite(Player *p) {
    LCD_Sprite(p->x, p->y, SPRITE_W, SPRITE_H, p->sprite, 7, p->frame, p->flip, 0);
}


void erase_player(Player *p) {
    FillRect(p->x, p->y, SPRITE_W, SPRITE_H, 0x0000); // negro
}

void DelaySoftware(uint32_t delay) {
    for (volatile uint32_t i = 0; i < delay * 8000; i++); // Aproximadamente 1 ms si el MCU corre a 72 MHz
}


void move_player(Player *p, char direction) {
    erase_player(p);  // Borra antes de mover
    if (movimiento_juego ==0){
    	switch(direction){
    	case '0':
    		PUNTOS_PARA_GANAR = PUNTOS_PARA_GANAR -1;
    		DelaySoftware(400);

    	break;
    	case '1':
    		PUNTOS_PARA_GANAR = PUNTOS_PARA_GANAR +1;
    		DelaySoftware(400);
    	break;
    	case '2':
    		startsequence = 0;
    		startsequence_2 = 1;
    		jugar = 1;
    	break;
    	}
    }
    if (movimiento_juego == 1){
    switch (direction) {
        case '0': case '5': p->y -= 5; break; // arriba
        case '1': case '6': p->y += 5; break; // abajo
        case '2': case '7': p->x -= 5; break; // izquierda
        case '3': case '8': p->x += 5; break; // derecha
        case '4': // Disparo hacia arriba
            if (!bullet1.active) {
                bullet1.x = p->x;
                bullet1.y = p->y - 25;
                bullet1.active = 1;
                bullet1.direction = -1;
            }
            break;
        case '9': // Disparo hacia arriba
                    if (!bullet2.active) {
                        bullet2.x = p->x;
                        bullet2.y = p->y + 25;
                        bullet2.active = 1;
                        bullet2.direction = 1;
                    }
                    break;


        }



    // Límites de pantalla
    if (p->x < 0) p->x = 0;
    if (p->x > 320 - SPRITE_W) p->x = 320 - SPRITE_W;
    if (p->y < 0) p->y = 0;
    if (p->y > 240 - SPRITE_H) p->y = 240 - SPRITE_H;

    draw_player_sprite(p);  // Redibuja en nueva posición
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART5) {
        switch (rx_data) {
            case '0': case '1': case '2': case '3': case '4':
                move_player(&player1, rx_data);
                break;

            case '5': case '6': case '7': case '8': case '9':
                move_player(&player2, rx_data);
                break;
            case 'a':
            	startsequence = 1;
            default:
                break;
        }

        HAL_UART_Receive_IT(&huart5, &rx_data, 1); // Reinicia recepción
    }
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_Clear(0x00);

//Este es el menu principal NO BORRAR
	LCD_Sprite(50,30,212,18,menutop,1,1,0,0);
	LCD_Sprite(110,90,98,42,menutitulo,1,1,0,0);
	LCD_Sprite(130,150,78,8,menujugadores,1,1,0,0);
	LCD_Sprite(30,180,254,12,menubottom1,1,1,0,0);
	LCD_Sprite(35,200,241,26,menubottom2,1,1,0,0);

	//Seccion de definicion de coordenadas iniciales
	posoriginalx = 160;
	uint16_t posoriginaly_1 = 220;
	uint16_t posoriginaly_2 = 0;




    // UART interrupciones
    HAL_UART_Receive_IT(&huart5, &rx_data, 1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (startsequence == 1){
			LCD_Clear(0x00);
		    LCD_Print("Vidas", 110, 80, 2, 0xffc0, 0);
		    while(jugar==0){
			LCD_Sprite(150,120,16,24,contadorinicial,5,PUNTOS_PARA_GANAR,0,0);
		    }
		    }
		if (startsequence_2 == 1){
			LCD_Clear(0x00);
			play = 1;
		    draw_player_sprite(&player1);
		    draw_player_sprite(&player2);

					FillRect(0, 0, 319, 239, 0x0000);
					//Seccion de spawn de los jugardores
					LCD_Sprite(150,220,19,20,nave_1,7,0,0,0);
					LCD_Sprite(150,10,19,20,nave_2,7,0,0,0);
					//Cuenta regresiva
					LCD_Sprite(150,120,16,24,contadorinicial,5,0,0,0);
					HAL_Delay(1000);
					LCD_Sprite(150,120,16,24,contadorinicial,5,1,0,0);
					HAL_Delay(1000);
					LCD_Sprite(150,120,16,24,contadorinicial,5,2,0,0);
					HAL_Delay(1000);
					LCD_Sprite(150,120,16,24,contadorinicial,5,3,0,0);
					HAL_Delay(1000);
					LCD_Sprite(150,120,16,24,contadorinicial,5,4,0,0);
					//Contadores de puntos
					LCD_Sprite(10,100,16,24,contadorinicial,5,3,0,0);
					LCD_Sprite(300,100,16,24,contadorinicial,5,3,0,0);
					startsequence_2 = 0;
					movimiento_juego = 1;
					//Musica de fondo
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		}

		// Detección de colisión
		if (bullet1.active) {
		    static uint32_t last_bullet_time = 0;
		    if (HAL_GetTick() - last_bullet_time > 10) {
		        FillRect(bullet1.x, bullet1.y, 18, 26, 0x0000); // Borra anterior
		        bullet1.y += 3 * bullet1.direction;

		        if (bullet1.y < 0 || bullet1.y > 240 - 26) {
		            bullet1.active = 0; // Sale de pantalla
		        } else {
		            int anim_bala1 = (bullet1.y / 10) % 7;
		            LCD_Sprite(bullet1.x, bullet1.y, 18, 26, bala_1, 3, anim_bala1, 0, 0);

		            // *** COLISIÓN CON PLAYER 2 ***
		            if (bullet1.x + 18 > player2.x && bullet1.x < player2.x + SPRITE_W &&
		                bullet1.y + 26 > player2.y && bullet1.y < player2.y + SPRITE_H) {
		                // Borra jugador dañado
		                erase_player(&player2);
		                // Explosión
		                animate_explosion(player2.x - 5, player2.y - 5);
		                LCD_Clear(0x00);
		                // Reinicia jugador en posición inicial
		                player1.x = 150;
		                player1.y = 220;
		                draw_player_sprite(&player1);
		                bullet2.active = 0;
		                player2.x = 150;
		                player2.y = 10;
		                draw_player_sprite(&player2);
		                bullet1.active = 0;
		                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
		                HAL_Delay(300);
		                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
		                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		                puntos_1 = puntos_1 - 1;
						LCD_Sprite(10,100,16,24,contadorinicial,5,puntos_1,0,0);
						LCD_Sprite(300,100,16,24,contadorinicial,5,puntos_2,0,0);
						if (puntos_1 <= PUNTOS_PARA_GANAR) {
						    LCD_Clear(0x0000);
						    LCD_Print("Jugador 1 GANA!", 40, 110, 2, 0xffc0,0);
						    HAL_Delay(3000);  // Espera 3 segundos
						    NVIC_SystemReset();  // Reinicia el microcontrolador
						}


		            }
		        }

		        last_bullet_time = HAL_GetTick();
		    }
		}
		if (bullet2.active) {
				    static uint32_t last_bullet_time = 0;
				    if (HAL_GetTick() - last_bullet_time > 10) {
				        FillRect(bullet2.x, bullet2.y, 18, 26, 0x0000); // Borra anterior
				        bullet2.y += 3 * bullet2.direction;

				        if (bullet2.y < 0 || bullet2.y > 240 - 26) {
				            bullet2.active = 0; // Sale de pantalla
				        } else {
				            int anim_bala1 = (bullet2.y / 10) % 7;
				            LCD_Sprite(bullet2.x, bullet2.y, 18, 26, bala_2, 3, anim_bala1, 0, 0);

				            // *** COLISIÓN CON PLAYER 2 ***
				            if (bullet2.x + 18 > player1.x && bullet2.x < player1.x + SPRITE_W &&
				                bullet2.y + 26 > player1.y && bullet2.y < player1.y + SPRITE_H) {
				                // Borra jugador dañado
				                erase_player(&player1);
				                // Explosión
				                animate_explosion(player1.x - 5, player1.y - 5);
				            	LCD_Clear(0x00);
				                // Reinicia jugador en posición inicial
				            	player2.x = 150;
				            	player2.y = 10;
				                draw_player_sprite(&player2);
				                bullet1.active = 0;
				                player1.x = 150;
				                player1.y = 220;
				                draw_player_sprite(&player1);
				                bullet2.active = 0;
				                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	       		                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
				                HAL_Delay(300);
				                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
				                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
				                puntos_2 = puntos_2 - 1;
								LCD_Sprite(300,100,16,24,contadorinicial,5,puntos_2,0,0);
								LCD_Sprite(10,100,16,24,contadorinicial,5,puntos_1,0,0);
								if (puntos_2 <= PUNTOS_PARA_GANAR) {
								    LCD_Clear(0x0000);
								    LCD_Print("Jugador 2 GANA!", 40, 110, 2, 0xffc0, 0);
								    HAL_Delay(3000);  // Espera 3 segundos
								    NVIC_SystemReset();  // Reinicia el microcontrolador
								}

				            }
				        }

				        last_bullet_time = HAL_GetTick();
				    }
				}



		play = 1;


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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
