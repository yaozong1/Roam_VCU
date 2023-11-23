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
#include "SEGGER_RTT.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_TxHeaderTypeDef TxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BufferSize 30
#define BufferSize_ESP 20
#define BufferSize_Sync 30
// CS宏定�??
#define W25N512_CS_LOW()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define W25N512_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define W25N_DEVICE_ID_READ_CMD    0x9F


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t TxData[8];//CAN BUS DATA
uint32_t TxMailbox;

uint8_t txData[] = "Hello, CANOK!";  // 要发送的数据
uint8_t txData_UART[] = "Hello, UART2!";  // 要发送的数据
uint8_t rxData[BufferSize];
uint8_t rxData_ESP[BufferSize_ESP];
uint8_t txData_ESP[BufferSize_Sync];
uint8_t rxData_cmp[] = {'f', 0x01, 0x02};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t W25N512GVEIG_ReadDeviceID(void);
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
	TxHeader.StdId = 0x01;
	TxHeader.DLC = 8;                 // 数据长度�??????????? 8 字节
	TxHeader.IDE = CAN_ID_STD;        // 使用标准标识�???????????
	TxHeader.RTR = CAN_RTR_DATA;      // 数据�???????????


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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET); //IMOUT 关闭
  SEGGER_RTT_printf(0, "IMOUT CLOSED ...\r\n");

  uint8_t deviceID = W25N512GVEIG_ReadDeviceID();

  bool IGN_PA0_O_first = 0;
  bool IGN_PA0_O_second = 0;
  IGN_PA0_O_first = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

  HAL_Delay(10);
  SEGGER_RTT_printf(0, "IGN_PA0_O_first Voltage: %d\r\n", (int)IGN_PA0_O_first);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  SEGGER_RTT_printf(0, "DEVICEID = %06X ", deviceID);

	  static bool uart1_pend = true;

	  //-------------------------------------------------USART1 ---------------------------------------------------//
  while(uart1_pend){
      // 等待接收数据，超时时间为 1000 毫秒，接收之前先清除�?有的缓存
	  memset(rxData, 0, BufferSize);

      HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, rxData, BufferSize, 5000);
      //rxData[BufferSize - 1] = '\0'; // 添加字符串终止符

      // �????查接收状�????
      if (status == HAL_OK) {
          // 成功接收到数�????
          // 在这里添加代码来处理接收到的数据
          // 你可以使�???? printf 或其他方式将接收到的数据显示出来

    	  int dataSize = sizeof(rxData);

    	  SEGGER_RTT_printf(0, "Uart1 DATA IS: ");
    	  for (int i = 0; i < dataSize; i++) {
    	      SEGGER_RTT_printf(0, "%02X ", rxData[i]);
    	  }
    	  SEGGER_RTT_printf(0, "\r\n");

    	    if (rxData[0] == 'c') {
    	        // 接收到了期望的字符串
    	    	SEGGER_RTT_printf(0, "GET THE Result DATA from NRF\r\n");

    	    	  if (deviceID == 0x20)
    	    			  {
    	    		  SEGGER_RTT_printf(0, "Get the right Device ID \r\n");
    	    		  rxData[7] = 0x11; //把VCU的flash结果也加�?
    //'c''modem result''sim result' 'aliyun result' 'motion result' 'nrfflash res' ''canbus result' 'vcu flash res'
    	    			  }

    	    	memcpy(TxData, rxData, 8);//把从nrf收过来的数据用CAN发�?�到ESP32

    	    	       if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK) {

    	    			  SEGGER_RTT_printf(0, "CAN_SENT OUT\r\n");
    	    		                                                                              }
    	                         }


      } else if (status == HAL_TIMEOUT) {
          // 超时，未接收到数�????
    	  //HAL_UART_Transmit(&huart1, txData, sizeof(txData), 1000);
    	  SEGGER_RTT_printf(0, "Uart1_LOOP DATA IS timeout \r\n");
          // 在这里可以添加�?�当的处理代�????
      } else {
          // 发生错误
    	  SEGGER_RTT_printf(0, "Uart1_LOOP DATA IS error \r\n");
          // 在这里可以添加�?�当的错误处理代�????
      }


	  if(rxData[0] != 'c' || rxData[BufferSize-1]!='s')
	  {
      while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
    	      SEGGER_RTT_printf(0, "Cleaning buffer for uart1");
		      uint8_t dummy;
		      HAL_UART_Receive(&huart1, &dummy, 1, 100);
		    }
	  }
	  if(rxData[0] == 'c' || rxData[BufferSize-1]=='s')
	  {
		  uart1_pend = false;

		  memcpy(txData_ESP, rxData, sizeof(rxData));
		  SEGGER_RTT_printf(0, "tx0 = %s \r\n", txData_ESP[0]);
		  SEGGER_RTT_printf(0, "tx29 = %s \r\n", txData_ESP[29]);
		  SEGGER_RTT_printf(0, "GET the right data from NRF");
	  }

  }
//-------------------------------------------------USART1 ---------------------------------------------------//

//两�?�之间的timeout现在是匹配好�?

//--------------------------------------------------USART3--------------------------------------------//
//uint8_t rxData_ESP[BufferSize_ESP];

	  do{

      // 等待接收数据，超时时间为 1000 毫秒
	  memset(rxData_ESP, 0, BufferSize_ESP);

      HAL_StatusTypeDef status = HAL_UART_Receive(&huart3, rxData_ESP, BufferSize_ESP, 1000);
      //rxData[BufferSize - 1] = '\0'; // 添加字符串终止符

      // �????查接收状�????
      if (status == HAL_OK) {
          // 成功接收到数�????

          // 在这里添加代码来处理接收到的数据
          // 你可以使�???? printf 或其他方式将接收到的数据显示出来

    	  IGN_PA0_O_second = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    	  HAL_Delay(10);
    	  SEGGER_RTT_printf(0, "IGN_PA0_O_second Voltage: %d\r\n", (int)IGN_PA0_O_second);
    	  if(IGN_PA0_O_second == 0 && IGN_PA0_O_first == 1 )
    	  {
    		  txData_ESP[27] = 0x50;
    		  SEGGER_RTT_printf(0, "IGN TEST PASS \r\n");
    	  }





    	  int dataSize = sizeof(rxData_ESP);

    	  SEGGER_RTT_printf(0, "Uart3 DATA IS: ");
    	  for (int i = 0; i < dataSize; i++) {
    	      SEGGER_RTT_printf(0, "%02X ", rxData_ESP[i]);
    	  }
    	  SEGGER_RTT_printf(0, "\r\n");

  	    if (rxData_ESP[0] == 'f'&& rxData_ESP[19] == 0x77) {
  	        // 接收到了期望的字符串
  	    	SEGGER_RTT_printf(0, "GET THE Final confirm from ESP32\r\n");

      	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET); //IMOUT 打开

      	    SEGGER_RTT_printf(0, "IMOUT OPEN ...\r\n");

      	    HAL_Delay(1000);

  	    	//memcpy(TxData, rxData, 8);//把从nrf收过来的数据用CAN发�?�到ESP32

  	    	       if (HAL_UART_Transmit(&huart3, txData_ESP, sizeof(txData_ESP), 2000) == HAL_OK) {

  	    			  SEGGER_RTT_printf(0, "IMEI&DEVICE ID_SENT OUT\r\n");
                                                           }

  	                         }

                            }


        else if (status == HAL_TIMEOUT) {
          // 超时，未接收到数�????
    	  SEGGER_RTT_printf(0, "Uart3_LOOP DATA IS timeout \r\n");
          // 在这里可以添加�?�当的处理代�????
      } else {
          // 发生错误
    	  SEGGER_RTT_printf(0, "Uart3_LOOP DATA IS error \r\n");
          // 在这里可以添加�?�当的错误处理代�????
      }
      HAL_Delay(10);
  }while(0);

	  //--------------------------------------------------USART3--------------------------------------------//




//	  HAL_Delay(100);  // 延迟 1000 毫秒，即 1 �???????????
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  CAN_FilterTypeDef  sFilterConfig;

    /* Configure the CAN Filter */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
   	 /* Filter configuration Error */
   	 Error_Handler();
    }

    /* Start the CAN peripheral */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
      /* Start Error */
      Error_Handler();
    }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 0;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

  // SPI CS Pin
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 设置默认为高
  W25N512_CS_HIGH();

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// 从W25N512GVEIG读取DEVICE ID的函�??
uint8_t W25N512GVEIG_ReadDeviceID(void)
{
    uint8_t device_id[4];
    device_id[3] = 0xFF;

    W25N512_CS_LOW();

    uint8_t cmd = W25N_DEVICE_ID_READ_CMD;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, device_id, 4, HAL_MAX_DELAY);

    W25N512_CS_HIGH();

   // return device_id[0];//return Dummy
   // return device_id[1];
   // return device_id[2];
    return device_id[3];

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
