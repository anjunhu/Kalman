/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "arm_math.h"
#include "lab1util.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
//#define LAB_PART_ONE 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef LAB_PART_ONE
  struct KalmanState ks5A = {
 		  .q = 0.1,
 		  .r = 0.9,
 		  .x = 5,
 		  .p = 0.1,
 		  .k = 0.0
   };
#endif

  //////////////////////////////////////// Assembly
  struct KalmanState ks5AC = {
 		  .q = 0.01,
 		  .r = 0.9,
 		  .x = 5,
 		  .p = 0.01,
 		  .k = 0.0
   };
  struct KalmanState ks50 = {
 		  .q = 0.01,
 		  .r = 0.9,
 		  .x = 50,
 		  .p = 0.01,
 		  .k = 0.0
   };
///////////////////////////////////////// Plain C
  struct KalmanState ks5C = {
   		  .q = 0.01,
   		  .r = 0.9,
   		  .x = 5,
   		  .p = 0.01,
   		  .k = 0.0
   };

///////////////////////////////////////// CIMSIS-DSP
   struct KalmanState ks5L = {
   		  .q = 0.01,
   		  .r = 0.9,
   		  .x = 5,
   		  .p = 0.01,
   		  .k = 0.0
   };


#ifdef LAB_PART_ONE
  struct KalmanState ksZero = {
		  .q = 0,
		  .r = 0,
		  .x = 5,
		  .p = 0,
		  .k = 0.0
  };
  struct KalmanState ksOverflow = {
  		  .q = 3e38,
  		  .r = 3e38,
  		  .x = 5,
  		  .p = 3e38,
  		  .k = 0.0
    };
  struct KalmanState ksUnderflow = {
  		  .q = -3e3,
  		  .r = 3e3,
  		  .x = 5,
  		  .p = 3e-38,
  		  .k = 0.0
    };
  float sanitychecker = 123.45;
  for (int i=0; i<5; i++) {
	 sanitychecker = kalmanUpdateA(&ks5A, i);
	 sanitychecker = kalmanUpdateC(&ks5C, i);
  }

  memset(output, 0, sizeof(output));
  resultholder = kalmanFilterA(TEST_ARRAY, output, &ks5A, 101);
  resultholder = kalmanFilterA(TEST_ARRAY, output, &ksZero, 101);
  resultholder = kalmanFilterA(TEST_ARRAY, output, &ksOverflow, 101);
  resultholder = kalmanFilterA(TEST_ARRAY, output, &ksUnderflow, 101);

  resultholder = kalmanFilterC(TEST_ARRAY, output, &ks5C, 101, 0);
  resultholder = kalmanFilterC(TEST_ARRAY, output, &ksZero, 101, 0);
  resultholder = kalmanFilterC(TEST_ARRAY, output, &ksOverflow, 101, 0);
  resultholder = kalmanFilterC(TEST_ARRAY, output, &ksUnderflow, 101, 0);

  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ksZero, 101, 0);
  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ksOverflow, 101, 0);
  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ksUnderflow, 101, 0);
#endif

  float TEST_ARRAY[101] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706, 10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493, 9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356, 9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039, 10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479, 10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721, 10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814, 10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024, 9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599, 10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259, 10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732, 10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876, 10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898, 10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583, 9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608, 10.3465431058, 9.98446410493, 9.79376005657, 10.202518901, 9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804, 10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909, 9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596, 10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178, 9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226, 10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928, 9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087, 9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486, 10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991, 9.5799256668};

  int resultholder = 12345;

  float output[101];


  memset(output, 0, sizeof(output));
  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ks5AC, 101, 1);

  memset(output, 0, sizeof(output));
  resultholder = kalmanFilterC(TEST_ARRAY, output, &ks5C, 101, 1);

  memset(output, 0, sizeof(output));
  resultholder = kalmanFilterL(TEST_ARRAY, output, &ks5L, 101, 1);

  float output50[101];
  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ks50, 101, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  ITM_Port32(31) = 111;
	  // A vs C with stats in C
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterAinC(TEST_ARRAY, output, &ks5AC, 101, 1);
	  }
	  ITM_Port32(31) = 222;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterC(TEST_ARRAY, output, &ks5AC, 101, 1);
	  }


	  // A vs C vs L with stats in L
	  ITM_Port32(31) = 333;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterAinL(TEST_ARRAY, output, &ks5AC, 101, 1);
	  }
	  ITM_Port32(31) = 444;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterCinL(TEST_ARRAY, output, &ks5C, 101, 1);
	  }
	  ITM_Port32(31) = 555;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterL(TEST_ARRAY, output, &ks5C, 101, 1);
	  }


	  // A vs C vs L, no stats
	  ITM_Port32(31) = 6666;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterAinL(TEST_ARRAY, output, &ks5AC, 101, 0);
	  }
	  ITM_Port32(31) = 7777;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterC(TEST_ARRAY, output, &ks5C, 101, 0);
	  }
	  ITM_Port32(31) = 8888;
	  for (int i=0; i<1000; i++){
		  resultholder = kalmanFilterL(TEST_ARRAY, output, &ks5C, 101, 0);
	  }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 60000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
