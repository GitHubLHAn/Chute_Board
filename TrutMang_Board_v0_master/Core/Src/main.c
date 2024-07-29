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
#include "st7920.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "rs485.h"
#include "user.h"


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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	uint16_t cnt_flag_led = 0;
	uint32_t tim_tick_1ms = 0;	
	
	uint32_t __s = 0, __e = 0, __i = 0;
	
	char ROW0[16], ROW1[16], ROW2[16], ROW3[16];
	
	extern rs485_t vRS485;
	extern const unsigned char viettel_post[];
	
	uint16_t cnt_flag_display_lcd = 0;
	uint8_t flag_send = 0;
	
	uint16_t cnt_flag_autosend = 0;
	
	uint8_t num_id = 1;
	
	uint16_t num_irq = 0;
	
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint32_t num_request_tx = 0, num_response_rx = 0, miss_mess = 0;
	uint16_t num_rx = 0;
	
	
	bool MASTER_OR_SLAVE = MASTER;
	
	uint16_t ID_BOARD_ARRAY[NUMBER_BOARD];
	uint16_t ID_need_send = 1;
	uint8_t isFull_ARRAY[NUMBER_BOARD];
	
	uint16_t num_packet = 0;
//	uint8_t num_packet_H = 0, num_packet_L = 0;
	uint8_t isFull = 0, isFullAll = 0;
	
	uint8_t flag_1ms = 0;
	uint8_t Status_Sensor = 0;
	uint32_t __s_sensor = 0;
	uint16_t cnt_sensor_on = 0;
	
	uint16_t cnt_thapled = 0, cnt_ledbao = 0;
	uint8_t onCoiBao = 0;
	
	uint8_t getButton = 0;
	uint16_t cnt_Button_on = 0;
	uint8_t isButton = 0;
	uint16_t cnt_isButton = 0;
	uint8_t isConfirmChute = 0;
	
	uint32_t cnt_free1 = 0, cnt_free2 = 0, cnt_free3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	void OnMaster(void);
	void OnSlave(void);

	uint32_t getMicroSecond(void){
		return tim_tick_1ms*1000 + __HAL_TIM_GetCounter(&htim2);
	}


	/*_____________________________________________________________*/
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  
			if(huart->Instance == USART1) 
			{    
				RS485_Read(&vRS485);
				HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1); 
				num_irq++;				
			}
		}
		
	/*_____________________________________________________________*/
		void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
			UNUSED(htim);
			
			if (htim->Instance == htim2.Instance){ 	// every 1ms			
				cnt_flag_led++;
				cnt_flag_autosend++;
				cnt_flag_display_lcd++;
				cnt_thapled++;
				cnt_ledbao++;
				
				flag_1ms = 1;
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	RS485_Init(&vRS485, &huart1);
	
	ID_BOARD_ARRAY[0] = vRS485.ID_Board; 
	for(uint8_t i=1; i<NUMBER_BOARD; i++){
		ID_BOARD_ARRAY[i] = i+300;
	}
	memset(isFull_ARRAY, 0x00, NUMBER_BOARD);
	onCoiBao = !HAL_GPIO_ReadPin(ADDR9_GPIO_Port, ADDR9_Pin);

	memset(ROW0, 0x00, 16);
	memset(ROW1, 0x00, 16);
	memset(ROW2, 0x00, 16);
	memset(ROW3, 0x00, 16);
	
	HAL_TIM_Base_Start_IT(&htim1);
	ST7920_Init();
	
	ST7920_GraphicMode(1);
	ST7920_DrawBitmap(viettel_post);
	
	HAL_Delay(2000);
	
	ST7920_GraphicMode(0);
	ST7920_Clear();
	
	sprintf(ROW0, "XIN CHAO");
	sprintf(ROW1, "MACH MANG");
	sprintf(ROW2, "ID: %d", vRS485.ID_Board);
	sprintf(ROW3, "VIETTEL POST");
	
	ST7920_SendString(0, 0, ROW0);
	ST7920_SendString(1, 0, ROW1);
	ST7920_SendString(2, 0, ROW2);
	ST7920_SendString(3, 0, ROW3);

	
	HAL_Delay(2000);
	HAL_GPIO_WritePin(LED_BAO_GPIO_Port,LED_BAO_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(THAPCOI_GPIO_Port,THAPCOI_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(THAPLED1_GPIO_Port,THAPLED1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(THAPLED2_GPIO_Port,THAPLED2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(THAPLED3_GPIO_Port,THAPLED3_Pin,GPIO_PIN_RESET);
	
	HAL_TIM_Base_Start_IT(&htim2);
	

	HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
	//--------------------------------------------------------------------	
		if(cnt_flag_autosend >= 100){
			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
			
			if(++ID_need_send >= NUMBER_BOARD)
				ID_need_send = 1;
		
			vRS485.txBuff[0] = vRS485.header;
			
			vRS485.txBuff[1] = (ID_BOARD_ARRAY[ID_need_send] >> 8) & 0xFF;
			vRS485.txBuff[2] = ID_BOARD_ARRAY[ID_need_send] & 0xFF;			
			vRS485.txBuff[3] = 0xAA;
			for(uint8_t i=4; i<SIZE_RS485-2; i++){
				vRS485.txBuff[i] = 0xFF;
			}
			vRS485.txBuff[SIZE_RS485-2] = rand() % 100;
			vRS485.txBuff[SIZE_RS485-1] = vRS485.ender;
			
			//HAL_Delay(1);
//			HAL_UART_Transmit(&huart1, vRS485.txBuff, SIZE_RS485, HAL_MAX_DELAY);
//			
//			//HAL_UART_Transmit(&huart1, (uint8_t*)"Hello", 5, HAL_MAX_DELAY);
//			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
//			num_request_tx++;
			
			vRS485.TxFlag = 1;
			cnt_flag_autosend = 0;
		}
		
		if(vRS485.TxFlag == 1){
				if(RS485_SendData(&vRS485, USART1, 2000)){		
					num_request_tx++;
					HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
				}
		}
	//--------------------------------------------------------------------
		if(vRS485.RxFlag == 1){   
			/*Handle Data received here*/
			num_response_rx++;
			uint16_t ID_board_slave = (vRS485.rxBuff[1] << 8) | vRS485.rxBuff[2];

			for(uint8_t i = 1; i<NUMBER_BOARD; i++){
				if(ID_BOARD_ARRAY[i] == ID_board_slave){
					isFull_ARRAY[i] = vRS485.rxBuff[4];
				}
			}

			vRS485.RxFlag = 0;
			vRS485.STATE = USART_WAIT;
		}
	//--------------------------------------------------------------------	
		if(cnt_flag_display_lcd >= 1000){
			cnt_flag_display_lcd = 0;
			
//			ST7920_GraphicMode(0);
//			ST7920_Clear();
			memset(ROW0, ' ', 16); 
			memset(ROW1, ' ', 16);
			memset(ROW2, ' ', 16);
			memset(ROW3, ' ', 16);
			
			ST7920_SendString(0, 0, ROW0);
			ST7920_SendString(1, 0, ROW1);
			ST7920_SendString(2, 0, ROW2);
			ST7920_SendString(3, 0, ROW3);

			sprintf(ROW0, "ID Master: %d", vRS485.ID_Board);
//			sprintf(ROW1, "Tx: %d, Rx: %d", num_request_tx, num_response_rx);
//			sprintf(ROW2, "So luong: %d", num_packet);
//			sprintf(ROW3, "Miss: %d", miss_mess);
			
			sprintf(ROW1, "So luong: %d ", num_packet);
			//sprintf(ROW1, "Rx: %d ", num_request_rx);
			if(isFull == 1){
				sprintf(ROW2, "Mang Day");
				sprintf(ROW3, "XXXXXXXXXXXXXXXX");
			}
			else{
				sprintf(ROW2, "Mang Chua Day");
				memset(ROW3, ' ', 16);
			}
			
			ST7920_SendString(0, 0, ROW0);
			ST7920_SendString(1, 0, ROW1);
			ST7920_SendString(2, 0, ROW2);
			ST7920_SendString(3, 0, ROW3);		
		}
	//--------------------------------------------------------------------
		if(cnt_flag_led >= 50){
			cnt_flag_led = 0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			miss_mess = num_request_tx - num_response_rx;
		}
		
	//--------------------------------------------------------------------
		Status_Sensor = HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin);
		getButton = !HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);
		if(flag_1ms == 1)
		{
			if(isConfirmChute == 0){
				if(Status_Sensor == 1){
					if(cnt_sensor_on == 2000){
						isFull = 1;
					}
					else{
						cnt_sensor_on++;
						if(cnt_sensor_on == 2)
							num_packet++;
					}
				}
				else{
					cnt_sensor_on = 0;
					isFull = 0;
				}
				isFull_ARRAY[0] = isFull;				
			}
			else{
			
			}
			
			if(getButton && isButton == 0){
				if(isFull == 1){
					if(cnt_Button_on == 100){
						isButton = 1;
						
						if(isConfirmChute == 0){
							isConfirmChute = 1;
							isFull_ARRAY[0] = 0;
						}
						else{
							isConfirmChute = 0;
							num_packet = 0;
						}
						cnt_free1++;
						
					}
					else{
						cnt_Button_on++;
					}
				}
				else{
					num_packet = 0;
				}
			}
			else{
				cnt_Button_on = 0;
			}
			if(isButton == 1){
				if(++cnt_isButton >= 3000){
					isButton = 0;
					cnt_isButton = 0;
				}
			}
			
			
			
			/*Check all slave be Full*/
			
			uint16_t check_full = 0;
			for(uint8_t i=0; i<NUMBER_BOARD; i++){
				check_full += isFull_ARRAY[i];
			}
			if(check_full == 0)
				isFullAll = 0;
			else
				isFullAll = 1;
			
			flag_1ms = 0;
		}
	//--------------------------------------------------------------------
		if(isFull == 1)
		{			
			if(isConfirmChute == 0){
				if(cnt_ledbao >= 500){
					cnt_ledbao = 0;
					HAL_GPIO_TogglePin(LED_BAO_GPIO_Port,LED_BAO_Pin);
				}
			}
			else{
				HAL_GPIO_WritePin(LED_BAO_GPIO_Port,LED_BAO_Pin,GPIO_PIN_SET);
			}	
		}
		else
		{
			HAL_GPIO_WritePin(LED_BAO_GPIO_Port,LED_BAO_Pin,GPIO_PIN_RESET);
		}
		
		
		if(isFullAll == 1){
		
			if(onCoiBao){
				HAL_GPIO_WritePin(THAPCOI_GPIO_Port,THAPCOI_Pin,GPIO_PIN_SET);
			}
			
			if(cnt_thapled >= 500){
				HAL_GPIO_TogglePin(THAPLED1_GPIO_Port,THAPLED1_Pin);
				HAL_GPIO_TogglePin(THAPLED2_GPIO_Port,THAPLED2_Pin);
				HAL_GPIO_TogglePin(THAPLED3_GPIO_Port,THAPLED3_Pin);
				cnt_thapled = 0;
			}
		}
		else{
			HAL_GPIO_WritePin(THAPCOI_GPIO_Port,THAPCOI_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(THAPLED1_GPIO_Port,THAPLED1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(THAPLED2_GPIO_Port,THAPLED2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(THAPLED3_GPIO_Port,THAPLED3_Pin,GPIO_PIN_RESET);
		}
		
		
	//--------------------------------------------------------------------
			
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, THAPCOI_Pin|THAPLED3_Pin|THAPLED2_Pin|THAPLED1_Pin
                          |LCD_RST_Pin|RS485_DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_SCK_Pin|LCD_MOSI_Pin|LED_BAO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : THAPCOI_Pin THAPLED3_Pin THAPLED2_Pin THAPLED1_Pin
                           LCD_RST_Pin RS485_DE_Pin */
  GPIO_InitStruct.Pin = THAPCOI_Pin|THAPLED3_Pin|THAPLED2_Pin|THAPLED1_Pin
                          |LCD_RST_Pin|RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR0_Pin ADDR1_Pin ADDR2_Pin ADDR3_Pin */
  GPIO_InitStruct.Pin = ADDR0_Pin|ADDR1_Pin|ADDR2_Pin|ADDR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR4_Pin ADDR5_Pin ADDR6_Pin ADDR7_Pin
                           ADDR8_Pin ADDR9_Pin BUTTON_Pin */
  GPIO_InitStruct.Pin = ADDR4_Pin|ADDR5_Pin|ADDR6_Pin|ADDR7_Pin
                          |ADDR8_Pin|ADDR9_Pin|BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_SCK_Pin LCD_MOSI_Pin LED_BAO_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_SCK_Pin|LCD_MOSI_Pin|LED_BAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*______________________________________________________________________________*/
	
	void OnMaster(void){
		if(flag_send == 1){
		
		}
	
	
	}
	
	
	
	
/*______________________________________________________________________________*/
	
	void OnSlave(void){
	
	
	
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
