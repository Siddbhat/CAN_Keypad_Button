/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
uint32_t TxMailbox;
char rxData[8];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int n=0;
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan1);
  /* USER CODE END 2 */

  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.DLC=8;
  TxHeader.ExtId=0x18A78F26;
  TxHeader.IDE=CAN_ID_EXT;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime=250;


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  CAN_TxHeaderTypeDef TxHeader1;
//  TxHeader1.DLC=8;
//  TxHeader1.ExtId=0x18A68F26;
//  TxHeader1.IDE=CAN_ID_EXT;
//  TxHeader1.RTR=CAN_RTR_DATA;
//  TxHeader1.TransmitGlobalTime=250;

  char TxData1[8]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData2[8]={0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData3[8]={0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData4[8]={0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData5[8]={0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData6[8]={0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData7[8]={0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData8[8]={0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00};
  char TxData9[8]={0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00};
  char TxData10[8]={0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00};
  char TxData11[8]={0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00};
  char TxData12[8]={0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00};
  char TxData13[8]={0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00};
  char TxData14[8]={0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00};

  char TxData16[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};
  char TxData17[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01};
  char TxData18[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00};
  char TxData19[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00};
  char TxData20[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00};
  char TxData21[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00};
  char TxData22[8]={0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00};
  char TxData23[8]={0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00};
  char TxData24[8]={0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00};
  char TxData25[8]={0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00};
  char TxData26[8]={0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00};
  char TxData27[8]={0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00};
  char TxData28[8]={0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00};
  char TxData29[8]={0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00};
  char TxData30[8]={0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00};
  char TxData15[8]={0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00};
  //char TxData[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

  //char TxData1[8]={0x80,0xAA,0xAA,0x00,0x00,0xA8,0x00,0x00};
  while (1)
  {
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData1, &TxMailbox) == HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData1, sizeof(TxData1), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData2, &TxMailbox) == HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData2, sizeof(TxData2), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData3, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData3, sizeof(TxData3), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData4, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData4, sizeof(TxData4), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData5, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData5, sizeof(TxData5), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData6, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData6, sizeof(TxData6), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData7, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData7, sizeof(TxData7), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData8, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData8, sizeof(TxData8), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData9, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData9, sizeof(TxData9), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData10, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData10, sizeof(TxData10), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData11, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData11, sizeof(TxData11), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData12, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData12, sizeof(TxData12), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData13, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData13, sizeof(TxData13), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData14, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData14, sizeof(TxData14), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}

		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData15, sizeof(TxData15), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData16, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData16, sizeof(TxData16), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData17, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData17, sizeof(TxData17), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData18, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData18, sizeof(TxData18), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData19, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData19, sizeof(TxData19), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData20, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData20, sizeof(TxData20), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData21, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData21, sizeof(TxData21), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData22, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData22, sizeof(TxData22), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData23, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData23, sizeof(TxData23), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData24, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData24, sizeof(TxData24), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData25, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData25, sizeof(TxData25), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData26, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData26, sizeof(TxData26), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData27, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData27, sizeof(TxData27), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData28, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData28, sizeof(TxData28), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData29, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData29, sizeof(TxData29), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData30, &TxMailbox)
				== HAL_OK)
			HAL_Delay(300);
		{
			HAL_UART_Transmit(&huart2, (uint8_t*) TxData30, sizeof(TxData30), 1000);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			HAL_Delay(300);
		}
		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData15, &TxMailbox)
						== HAL_OK)
					HAL_Delay(300);
	}
//	 // HAL_CAN_AddTxMessage(&hcan1,&TxHeader,TxData,&TxMailbox);
//
//	  if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader1,TxData1,&TxMailbox) == HAL_OK)
//	  	  	  {
//	  	  		  HAL_UART_Transmit(&huart2,(uint8_t *)TxData1, sizeof(TxData1),1000);
//	  	  	                       HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
//	  	  	                       HAL_Delay(300);
//	  	  	  }         //  HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  //__HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
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
