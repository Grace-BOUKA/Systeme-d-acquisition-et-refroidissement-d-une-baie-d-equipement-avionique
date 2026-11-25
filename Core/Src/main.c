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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include<string.h>
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONSIGNE_CAPTEUR1 25
#define CONSIGNE_CAPTEUR2 30

#define COMMANDE_OK (1<<1)
#define TELEMETRIE_OK (2<<1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* Definitions for TaskCommande */
osThreadId_t TaskCommandeHandle;
const osThreadAttr_t TaskCommande_attributes = {
  .name = "TaskCommande",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskTelemetrie */
osThreadId_t TaskTelemetrieHandle;
const osThreadAttr_t TaskTelemetrie_attributes = {
  .name = "TaskTelemetrie",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskIHM */
osThreadId_t TaskIHMHandle;
const osThreadAttr_t TaskIHM_attributes = {
  .name = "TaskIHM",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PipeQueue */
osMessageQueueId_t PipeQueueHandle;
const osMessageQueueAttr_t PipeQueue_attributes = {
  .name = "PipeQueue"
};
/* Definitions for Mutexe_UART */
osMutexId_t Mutexe_UARTHandle;
const osMutexAttr_t Mutexe_UART_attributes = {
  .name = "Mutexe_UART"
};
/* Definitions for EventIHM */
osEventFlagsId_t EventIHMHandle;
const osEventFlagsAttr_t EventIHM_attributes = {
  .name = "EventIHM"
};
/* USER CODE BEGIN PV */

//-------> Informations sur la chaîne Acquistion-Commande PWM
typedef struct{
	uint16_t capteur1; // Jumeau Température
	uint16_t capteur2; // Jumeau humidité
	uint16_t duty_PWM;  // Rapport cyclique du PWM
	uint8_t  fsm_state; // 0: Normal, 1: Warning, 2: Critical
	uint32_t time_systeme;      // Temps système (ms)
}info_commande_t;

info_commande_t data_commande={
		.time_systeme=0,
		.capteur1=0,
		.capteur2=0,
		.duty_PWM=0,
		.fsm_state=0,
};

uint8_t flag_IWGD=0;


//--------> Variables buffer pour l'informations via la télémetrie
char buffer_UART[500];  // télémetrie via UART
volatile uint16_t buffer_capteur[2]; // Lecture des potentiomètres (pour simuler la température et humidité dans la baie avionique)

// Efface l'écran et remet le curseur en haut (sur Tera Term via UART)
char clear_screen[] = "\033[2J\033[H";
char message_erreur[]="--->\033[31m SYSTEME KO (Non respect écheance de tâche)\033[0m \r\n	\033[1m (Attente reset du Watchdog)\033[0m\r\n";


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_IWDG_Init(void);
void StartTaskCommande(void *argument);
void StartTaskTelemetrie(void *argument);
void StartTaskIHM(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//------> Indispensable to use iprintf and others on serial debug terminal
int __io_putchar(int ch)
{
	// Send the character to ITM canal
	ITM_SendChar(ch);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

//  	HAL_ADC_Start_DMA(&hadc3,(uint32_t *)buffer_capteur, 2);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutexe_UART */
  Mutexe_UARTHandle = osMutexNew(&Mutexe_UART_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of PipeQueue */
  PipeQueueHandle = osMessageQueueNew (10, sizeof(info_commande_t), &PipeQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskCommande */
  TaskCommandeHandle = osThreadNew(StartTaskCommande, NULL, &TaskCommande_attributes);

  /* creation of TaskTelemetrie */
  TaskTelemetrieHandle = osThreadNew(StartTaskTelemetrie, NULL, &TaskTelemetrie_attributes);

  /* creation of TaskIHM */
  TaskIHMHandle = osThreadNew(StartTaskIHM, NULL, &TaskIHM_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of EventIHM */
  EventIHMHandle = osEventFlagsNew(&EventIHM_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_vert_Pin|led_rouge_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_vert_Pin led_rouge_Pin */
  GPIO_InitStruct.Pin = led_vert_Pin|led_rouge_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskCommande */
/**
  * @brief  Function implementing the TaskCommande thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskCommande */
void StartTaskCommande(void *argument)
{
  /* USER CODE BEGIN 5 */

	//----> Demarrage du PWM de la commande
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	TickType_t pause_commande=osKernelGetTickCount(); // Variable pour la pause absolue

  /* Infinite loop */
  for(;;)
  {
	pause_commande+=pdMS_TO_TICKS(10); // Echéance de la tâche (Freq= 100 Hz)

//--------------> Lecture des capteurs (Lecture ADC des potentiometres)
	HAL_ADC_Start_DMA(&hadc3,(uint32_t *)buffer_capteur, 2);
	taskENTER_CRITICAL();
	data_commande.capteur1=(100*buffer_capteur[0])/4095;
	data_commande.capteur2=(100*buffer_capteur[1])/4095;
	taskEXIT_CRITICAL();

//-------------> Loi de commande du signal de commande PWM par machine à état

	if(data_commande.capteur1 <= 45)
	{
		TIM1->CCR1=30; // Regime normale du moteur
	}
	else if(data_commande.capteur1 > 45 && data_commande.capteur1 <60)
	{
		TIM1->CCR1=70; // Zone de danger
	}
	else
	{
		TIM1->CCR1=90; // Plein régime
	}

	data_commande.duty_PWM=TIM1->CCR1;

	//-------------> Synchronisation par Fil de message (Queue)
	osMessageQueuePut(PipeQueueHandle, &data_commande, 0, pdMS_TO_TICKS(10));

	// ----------> Simulation non respect d'échéance de la tâche
	if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)==GPIO_PIN_RESET)
	{
		HAL_Delay(20);
	}

	// Check sur deadline de la tâche
	if(osKernelGetTickCount() <= pause_commande) //Diagnostique sur l'échéance de la tâche
	{
		osEventFlagsSet(EventIHMHandle, COMMANDE_OK);
	}

    osDelayUntil(pause_commande);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskTelemetrie */
/**
* @brief Function implementing the TaskTelemetrie thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTelemetrie */
void StartTaskTelemetrie(void *argument)
{
  /* USER CODE BEGIN StartTaskTelemetrie */
	TickType_t pause_telemetrie=osKernelGetTickCount();
	info_commande_t rec_commande;

	HAL_UART_Transmit(&huart2, (uint8_t*)clear_screen, strlen(clear_screen), pdMS_TO_TICKS(1)); // Efface l'ecran

  /* Infinite loop */
  for(;;)
  {



	pause_telemetrie+=pdMS_TO_TICKS(100); // Echéance Tâche Télémetrie

	//----------> Synchronisation via Queue de message
	if(osMessageQueueGet(PipeQueueHandle, &rec_commande, 0, pdMS_TO_TICKS(10))==osOK)
	{
		if(flag_IWGD==0)
		{
	//----------> Envoie Télémetrie
		rec_commande.time_systeme=osKernelGetTickCount(); // Horodatage
		HAL_UART_Transmit(&huart2, (uint8_t*)clear_screen, strlen(clear_screen), pdMS_TO_TICKS(1)); // Efface l'ecran
		sprintf(buffer_UART,"\033[1m === STATION SOL - BAIE AVIONIQUE ===\033[0m \r\n"
				"------------------------------------------\r\n"
				"\033[1m ------> [CAPTEURS & COMMANDE]	\033[0m\r\n"
				"\033[33m Température \033[0m : %u °C\r\n"
				"\033[33m Humidité \033[0m: %u %%\r\n"
				"\033[33m Rapport cyclique PWM \033[0m: %u \r\n"
				"------------------------------------------\r\n"
				"\033[1m ------> [SECURITE & TEMPS REEL]	\033[0m\r\n"
				"\033[33m Horloge_systeme \033[0m: %lu ms\r\n"
				"\033[33m Etat du système \033[0m: \033[32m OK \033[0m\r\n"
				"\033[1m======================================\033[0m \r\n\n",
				rec_commande.capteur1,
				rec_commande.capteur2,
				rec_commande.duty_PWM,
				rec_commande.time_systeme);

		HAL_UART_Transmit(&huart2, (uint8_t *)buffer_UART, sizeof(buffer_UART), HAL_MAX_DELAY);


		}

	}

	else
	{
		iprintf("----> Echec envoie vers Télémetrie\n");

	}

//----> Check sur deadline de la tâche Télémetrie
	if(osKernelGetTickCount() <= pause_telemetrie)
	{
		osEventFlagsSet(EventIHMHandle, TELEMETRIE_OK);
	}

    osDelayUntil(pause_telemetrie);
  }


  /* USER CODE END StartTaskTelemetrie */
}

/* USER CODE BEGIN Header_StartTaskIHM */
/**
* @brief Function implementing the TaskIHM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskIHM */
void StartTaskIHM(void *argument)
{
  /* USER CODE BEGIN StartTaskIHM */
	TickType_t pause_IHM=osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  pause_IHM+=pdMS_TO_TICKS(100); // ---> Cycle de verification d'intégrité du système


	  uint32_t check= osEventFlagsWait(EventIHMHandle, COMMANDE_OK | TELEMETRIE_OK, osFlagsWaitAll, pdMS_TO_TICKS(100)); // Vérification respect Echéance

	  if(!((check & osFlagsError)))
	  {
		  HAL_GPIO_WritePin(led_vert_GPIO_Port, led_vert_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_RESET);

		  // Affiche le tableau de bord statique
		  iprintf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		  iprintf("===============================\r\n");
		  iprintf("      (IHM) TABLEAU DE BORD     \r\n");
		  iprintf("===============================\r\n");
		  iprintf(" Temperature (Capteur 1) : %3u        \r\n",data_commande.capteur1);
		  iprintf(" Humidite (Capteur 2) : %3u  \r\n",data_commande.capteur2);
		  iprintf(" Rapport cyclique_PWM: %3u  \r\n",data_commande.duty_PWM);
		  iprintf(" Tick_systeme: %lu	\r\n",osKernelGetTickCount());
		  iprintf(" Etat du systeme: OK\r\n");
		  iprintf("===============================\r\n");

		  HAL_IWDG_Refresh(&hiwdg); // Nourrir le watchdog
		  flag_IWGD=0;
	  }

	  else
	  {
		  iprintf("=====-----> SYSTEME_KO\n");
		  HAL_GPIO_WritePin(led_vert_GPIO_Port, led_vert_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(led_rouge_GPIO_Port, led_rouge_Pin, GPIO_PIN_SET);

		  HAL_UART_Transmit(&huart2, (uint8_t*)clear_screen, strlen(clear_screen), HAL_MAX_DELAY);  // Efface l'ecran
		  HAL_UART_Transmit(&huart2, (uint8_t*)message_erreur, sizeof(message_erreur), HAL_MAX_DELAY);

		  flag_IWGD=1;

	  }

    osDelayUntil(pause_IHM);
  }
  /* USER CODE END StartTaskIHM */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
