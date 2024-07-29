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
#include "stdlib.h"
#include <math.h>
#include "rs485.h"


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
	
	char ROW0[16], ROW1[16], ROW2[16], ROW3[16];
	
	extern rs485_t vRS485;
	extern const unsigned char viettel_post[];
	
	uint16_t flag_display_lcd = 0;
	
	uint16_t cnt_flag_autosend = 0;
	
	uint8_t num_id = 1;
	
	uint16_t num_irq = 0;
	
	uint8_t sensor_status = 0;
	
	uint32_t start;
	
	uint16_t time_alarm = 2000;
	
	uint8_t led_status;
	
	uint8_t flag_sensor;
	
	uint16_t cnt_RS485_send;
	
	uint8_t hour = 0;
	uint8_t minute = 0;
	uint32_t danhan = 0, dagui = 0, miss_mess = 0;
	uint16_t num_rx = 0;
	
	uint16_t address_chute;
	
	uint16_t quantity=999;
	
	uint8_t a0,a1,a2,a3,a4,a5,a6,a7,a8,a9;
	
	uint8_t rxbuff[16];
	
	uint8_t ID_chute = 0x01;
	
	uint8_t quanty_sp;
	
	uint8_t cnt_ID = 0;
	
	char *chute_status;
	
	uint16_t address_chute = 1;

/* USER CODE END PV */

/* Priv	ate function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

		void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  
			if(huart->Instance == USART1) 
			{    
				RS485_Read(&vRS485);
				HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1); 			
			}
		}

	/*_____________________________________________________________*/
//	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {  
//			if(huart->Instance == USART1) 
//			{    
//				RS485_Read(&vRS485);
//				HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1); 
//				num_irq++;				
//			}
//		}
		
	/*_____________________________________________________________*/
		void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
			UNUSED(htim);
			
			if (htim->Instance == htim2.Instance){ 	// every 1ms			
				cnt_flag_led++;
				cnt_flag_autosend++;
				flag_display_lcd++;
				cnt_RS485_send++;
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
	RS485_Init(&vRS485,&huart1);
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	
	memset(ROW0, 0x00, 16);
	memset(ROW1, 0x00, 16);
	memset(ROW2, 0x00, 16);
	memset(ROW3, 0x00, 16);
	
	ST7920_Init();
	
	ST7920_GraphicMode(1);
	ST7920_DrawBitmap(viettel_post);
	
	HAL_Delay(2000);
	
	ST7920_GraphicMode(0);
	ST7920_Clear();
	
//	sprintf(ROW0, "ID: %d", vRS485.ID_Board);
//	sprintf(ROW1, "LEHUUAN - 457735");
//	sprintf(ROW2, "VIETTEL POST");
//	sprintf(ROW3, "THEOCACHCUABAN");
	
//	ST7920_SendString(0, 0, ROW0);
//	ST7920_SendString(1, 0, ROW1);
//	ST7920_SendString(2, 0, ROW2);
//	ST7920_SendString(3, 0, ROW3);

//	HAL_UART_Receive_IT(&huart1, &vRS485.rxByte, 1);
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		a0 = HAL_GPIO_ReadPin(ADDR0_GPIO_Port,ADDR0_Pin);
//		a1 = HAL_GPIO_ReadPin(ADDR1_GPIO_Port,ADDR1_Pin);
//		a2 = HAL_GPIO_ReadPin(ADDR2_GPIO_Port,ADDR2_Pin);
//		a3 = HAL_GPIO_ReadPin(ADDR3_GPIO_Port,ADDR3_Pin);
//		a4 = HAL_GPIO_ReadPin(ADDR4_GPIO_Port,ADDR4_Pin);
//		a5 = HAL_GPIO_ReadPin(ADDR5_GPIO_Port,ADDR5_Pin);
//		a6 = HAL_GPIO_ReadPin(ADDR6_GPIO_Port,ADDR6_Pin);
//		a7 = HAL_GPIO_ReadPin(ADDR7_GPIO_Port,ADDR7_Pin);
//		a8 = HAL_GPIO_ReadPin(ADDR8_GPIO_Port,ADDR8_Pin);
//		a9 = HAL_GPIO_ReadPin(ADDR9_GPIO_Port,ADDR9_Pin);
//		
//		address_chute = a0|a1<<1|a2<<2|a3<<3|a4<<4|a5<<5|a6<<6|a7<<7|a8<<8|a9<<9;
//		
//		sensor_status = HAL_GPIO_ReadPin(SENSOR_GPIO_Port,SENSOR_Pin);
//		
//		if(sensor_status == 1)
//		{
//				flag_sensor = 1;
//		}
//		else 
//		{
//			flag_sensor = 0;
//			start = HAL_GetTick();
//		}	
//		if(flag_sensor == 1)
//		{
//			if(HAL_GetTick() - start < 0)
//			{
//				uint16_t time_alarm1;
//				time_alarm1 = pow(2,32)-1-start+HAL_GetTick();
//				if(time_alarm1 >= time_alarm)
//				{
//					led_status = 1;
//				}
//			}
//			if(HAL_GetTick() - start >= time_alarm)
//			{
//				led_status = 1;
//			}
//		}
//		
//		if(flag_sensor == 0)
//		{
//				led_status = 0;
//		}
//			
//		if(led_status==1)
//		{
//			chute_status = "day";
//			HAL_GPIO_WritePin(LED_BAO_GPIO_Port,LED_BAO_Pin,GPIO_PIN_SET);
////			HAL_GPIO_WritePin(THAPCOI_GPIO_Port,THAPCOI_Pin,GPIO_PIN_SET);
//			HAL_GPIO_TogglePin(THAPLED1_GPIO_Port,THAPLED1_Pin);
//			HAL_GPIO_TogglePin(THAPLED2_GPIO_Port,THAPLED2_Pin);
//			HAL_GPIO_TogglePin(THAPLED3_GPIO_Port,THAPLED3_Pin);
//			HAL_Delay(500);
//		}
//		else
//		{
//			chute_status="chua day";
//			HAL_GPIO_WritePin(LED_BAO_GPIO_Port,LED_BAO_Pin,GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(THAPCOI_GPIO_Port,THAPCOI_Pin,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(THAPLED1_GPIO_Port,THAPLED1_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(THAPLED2_GPIO_Port,THAPLED2_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(THAPLED3_GPIO_Port,THAPLED3_Pin,GPIO_PIN_RESET);
//		}
		
		if(cnt_RS485_send>=1000)
		{		
			cnt_RS485_send = 0;
			cnt_ID++;
			
			HAL_GPIO_WritePin(RS485_DE_GPIO_Port,RS485_DE_Pin,GPIO_PIN_SET);
			vRS485.txBuff[0] = 0xFE;
			vRS485.txBuff[1] = 0xFE; 
			vRS485.txBuff[2] = cnt_ID>>8&0xFF;
			vRS485.txBuff[3] = cnt_ID&0xFF;
			vRS485.txBuff[4] = quanty_sp; // so luong san pham
			vRS485.txBuff[5] = 0x00; 
			vRS485.txBuff[6] = 0x00;
			vRS485.txBuff[7] = 0x00;
			vRS485.txBuff[8] = 0x00;
			vRS485.txBuff[9] = 0x00;
			vRS485.txBuff[10] =	0x00;
			vRS485.txBuff[11] = 0x00;
			vRS485.txBuff[12] = 0x00;
			vRS485.txBuff[13] = 0x00;
			vRS485.txBuff[14] = 0xFD;
			vRS485.txBuff[15] = 0xFF;	
			
			if(cnt_ID == 7)
			{
				cnt_ID = 0;
			}
			
			vRS485.TxFlag = 1;
		}
		
		if(vRS485.RxFlag == 1){   
			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
			vRS485.RxFlag = 0;   
				
			//Xu ly data nhan duoc o day
			for(int i=0; i<16; i++){
					rxbuff[i] = vRS485.rxBuff[i];
			}			
			//for(uint16_t index=0; index<1000; index++);			//delay with for 200us 
			vRS485.STATE = USART_WAIT;
		}
		
		if(vRS485.TxFlag == 1)
			{				
				RS485_SendData(&vRS485, USART1, 2000);
			}
		
		if(flag_display_lcd >= 1000){
			flag_display_lcd = 0;
			
		//	address_chute = (rxbuff[3]<<8)|rxbuff[4];
			
			ST7920_GraphicMode(0);
			ST7920_Clear();
			//HAL_Delay(5);
			
			sprintf(ROW0, "Hello world");
			sprintf(ROW1, "%u",vRS485.cnt_mess_sent);
			sprintf(ROW2, "%u",vRS485.cnt_mess_received);
			
			ST7920_SendString(0, 0, ROW0);
			ST7920_SendString(1, 0, ROW1);
			ST7920_SendString(2, 0, ROW2);
			ST7920_SendString(3, 0, ROW3);
		}
		
//		if(cnt_flag_autosend >= 10){
//			cnt_flag_autosend = 0;
//			HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
//			
//			vRS485.txBuff[0] = 0xFE;
//			vRS485.txBuff[1] = 0x00;
//			
//			if(num_id == 2)
//				vRS485.txBuff[2] = 0x02;
//			else if(num_id == 3)
//				vRS485.txBuff[2] = 0x03;
//			else if(num_id == 4)
//				vRS485.txBuff[2] = 0x04;
//			else
//				vRS485.txBuff[2] = 0xFF;
//			
//			if(++num_id > 4)
//				num_id = 2;
//			
//			
//			vRS485.txBuff[3] = rand() % 100;
//			for(uint8_t i = 4; i< SIZE_RS485 - 1; i++)
//				vRS485.txBuff[i] = 0xFF;
//			vRS485.txBuff[SIZE_RS485 - 1] = 0xFF;
//			
//			HAL_Delay(5);

//			
//			HAL_UART_Transmit(&huart1, vRS485.txBuff, SIZE_RS485, HAL_MAX_DELAY);
//			//HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET); 
//			vRS485.cnt_mess_send++;
//			flag_display_lcd = 1;
//		
//		}
		
		
		if(cnt_flag_led >= 100){
			cnt_flag_led = 0;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		}
		
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
