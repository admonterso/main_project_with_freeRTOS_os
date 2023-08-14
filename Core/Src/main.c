/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "extra_functions_for_work.h"
#include "quectel_commands_shortand.h"
#include "rc522.h"
#include "liquidcrystal_i2c.h"
#include "GEOtextLib.h"
#include "switchSounds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint8_t store_input_flag = 0;
uint8_t c;
uint32_t version;
uint64_t terminalID;
char * terminalStr;
uint8_t MQTT_CHECK_DATA[100];
uint8_t BUFFER[50];
int LENGTH;
int i = 0;
int check = 0;
uint8_t postData[250];
SemaphoreHandle_t semaphore_to_enable_status_process;
SemaphoreHandle_t semaphore_to_do_post;


//for trecking space taken by tasks
UBaseType_t uxHighWaterMark_for_card_read;
UBaseType_t uxHighWaterMark_for_process_status;
UBaseType_t uxHighWaterMark_for_post_mqtt;
//for trecking space taken by tasks

//events
EventGroupHandle_t status_event;
EventBits_t event_bits;
//events
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define versionAdress 0x0800BFF0
#define resetAddress 0x08000000
#define currentTerminalADRR 0x0800B000

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define card_read_allowed ( 1UL << 0UL )
#define request_make_request_allowed ( 1UL << 1UL )
#define card_read_NOT_allowed ( 1UL << 2UL )

const EventBits_t all_bits = ( card_read_NOT_allowed | request_make_request_allowed | card_read_allowed );
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId read_cardsHandle;
osThreadId process_statusHandle;
osThreadId postenabledHandle;
osThreadId checkHandle;
osSemaphoreId semaphore_to_do_postHandle;
/* USER CODE BEGIN PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

BaseType_t xHigherPriorityTaskWoken;

  if (huart->Instance == USART1)
  {
	xHigherPriorityTaskWoken = pdFALSE;
	if(c == '<'){
		store_input_flag = 1;
	}

	if(store_input_flag){
		BUFFER[i++] = c;
	}
	if(c == '!'){
		store_input_flag = 0;


		xSemaphoreGiveFromISR(semaphore_to_enable_status_process, &xHigherPriorityTaskWoken );

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		//HAL_UART_Transmit(&huart1, BUFFER, strlen((char *)BUFFER), 50);
	}
	  HAL_UART_Receive_IT(&huart1, &c, 1);
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void read_card_task(void const * argument);
void process_status_task(void const * argument);
void send_card_data_MQTT(void const * argument);
void check_MQTT(void const * argument);

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
	__disable_irq();
	SCB->VTOR = 0x800C000;
	__enable_irq();
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HD44780_Init(2);


  HAL_UART_Receive_IT(&huart1, &c, 1);

  version = *(__IO uint32_t *)versionAdress; // for version check
  terminalID = *(uint64_t *)currentTerminalADRR;
  terminalStr = convertNumberToCharArray(terminalID);
//
 sprintf((char*) MQTT_CHECK_DATA, "{\"operationType\":\"check\",\"content\":{\"terminalID\":\"%s\",\"firmwareVersion\":%ld}}",terminalStr, version);


  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of semaphore_to_do_post */
  osSemaphoreDef(semaphore_to_do_post);
  semaphore_to_do_postHandle = osSemaphoreCreate(osSemaphore(semaphore_to_do_post), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  semaphore_to_enable_status_process =  xSemaphoreCreateBinary();
  semaphore_to_do_post = xSemaphoreCreateBinary();


  status_event = xEventGroupCreate();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of read_cards */
  osThreadDef(read_cards, read_card_task, osPriorityNormal, 0, 254);
  read_cardsHandle = osThreadCreate(osThread(read_cards), NULL);

  /* definition and creation of process_status */
  osThreadDef(process_status, process_status_task, osPriorityAboveNormal, 0, 200);
  process_statusHandle = osThreadCreate(osThread(process_status), NULL);

  /* definition and creation of postenabled */
  osThreadDef(postenabled, send_card_data_MQTT, osPriorityAboveNormal, 0, 128);
  postenabledHandle = osThreadCreate(osThread(postenabled), NULL);

  /* definition and creation of check */
  osThreadDef(check, check_MQTT, osPriorityNormal, 0, 254);
  checkHandle = osThreadCreate(osThread(check), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  printMiadetBarati(0, 2);

  xEventGroupSetBits(status_event, card_read_allowed);
  event_bits = xEventGroupGetBits(status_event);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hi2c1.Init.OwnAddress1 = 0;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin RELAY_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_read_card_task */
/* USER CODE END Header_read_card_task */
void read_card_task(void const * argument)
{
  /* USER CODE BEGIN 5 */


	for(;;){

//		 MQTTPubToTopic(strlen((char*)MQTT_CHECK_DATA));
//		 HAL_UART_Transmit(&huart1, MQTT_CHECK_DATA, strlen((char*)MQTT_CHECK_DATA), 50);
//
//		 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//
		xEventGroupWaitBits(status_event, card_read_allowed, pdFALSE, pdTRUE, portMAX_DELAY);
		event_bits = xEventGroupGetBits(status_event);
		uint8_t card_read_stat = cardOperationWithBlockedSector(postData);
		 //printMiadetBarati(0, 2);
		uxHighWaterMark_for_card_read = uxTaskGetStackHighWaterMark( NULL );

		 if(card_read_stat){
			 uint8_t bpundCount = 0;
			 uint8_t postEnable = 0;

			 LENGTH = strlen((char*)postData);

			 for(int i = 0; i< LENGTH+5; i++){

				 if(postData[i]== '{' || postData[i] == '}'){
					 bpundCount++;
				 }

			 }
			 if(bpundCount != 4){
				postEnable = 1;
				memset(postData, 0, sizeof(postData));
			  }

			 if(postEnable == 0){
				 CardReadSound();
				 uxHighWaterMark_for_card_read = uxTaskGetStackHighWaterMark( NULL );
				 //xEventGroupSetBits(status_event, card_read_NOT_allowed);
				 xEventGroupSetBits(status_event,request_make_request_allowed);
				 event_bits = xEventGroupGetBits(status_event);

			 }

		 }
		 osDelay(100);
		}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_process_status_task */
void vApplicationIdleHook( void )
{
 /* This hook function does nothing but increment a counter. */
}

/* USER CODE END Header_process_status_task */
void process_status_task(void const * argument)
{
  /* USER CODE BEGIN process_status_task */
	for(;;){
		xSemaphoreTake( semaphore_to_enable_status_process, portMAX_DELAY );
		uxHighWaterMark_for_process_status = uxTaskGetStackHighWaterMark( NULL );
		int Status = takeStatus(BUFFER, i);
		i = 0;
		uint8_t dispData[50];
		//HAL_UART_Transmit(&huart1, BUFFER, sizeof BUFFER / sizeof BUFFER[0], 10);
		switch(Status){
			case 202:
				HAL_GPIO_TogglePin(GPIOB, RELAY_Pin);
				AppruveSound();
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 1);


				prinWarmateba(0, 3);
				insert(postData);
				uxHighWaterMark_for_process_status = uxTaskGetStackHighWaterMark( NULL );
				LENGTH = strlen((char*)postData);
				MQTTPubToTopic(LENGTH);
				HAL_Delay(10);
				HAL_UART_Transmit(&huart1, postData, LENGTH, 10);

				break;
			case 200:
				//AppruveSound();
				check++;
				break;
			case 201:
				//HAL_GPIO_TogglePin(GPIOB, RELAY_Pin);

				takeData(BUFFER, strlen((char*)BUFFER), dispData);
				printBalansi(0, 0);
				HD44780_PrintStr((char*) dispData);
				HAL_Delay(1000);
				HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);
				HAL_Delay(1000);
				printMiadetBarati(0, 2);
				memset(postData, 0, sizeof(postData));
				uxHighWaterMark_for_process_status = uxTaskGetStackHighWaterMark( NULL );
				xEventGroupClearBits(status_event, all_bits);
				xEventGroupSetBits(status_event, card_read_allowed);
				break;
			case 293:

				//takeData(buffer, count, dispData);
				HD44780_Clear();
				HD44780_SetCursor(0, 0);
				printUcxoBaratia(0,0);
				ErrorSound();
				HAL_Delay(1000);
				printMiadetBarati(0, 2);
				break;
			case 291:
				//takeData(buffer, count, dispData);
				HD44780_Clear();
				HD44780_SetCursor(0, 0);
				printBlansiAraa(0, 0);
				ErrorSound();
				HAL_Delay(1000);
				printMiadetBarati(0, 2);
				break;
			case 296:
				//takeData(buffer, count, dispData);
				HD44780_Clear();
				HD44780_SetCursor(0, 0);
				HD44780_PrintStr((char*) dispData);
				ErrorSound();
				HAL_Delay(3000);
				printMiadetBarati(0, 2);
				break;
			case 297:
				//takeData(buffer, count, dispData);
				HD44780_Clear();
				HD44780_SetCursor(0, 0);
				HD44780_PrintStr((char*) dispData);
				ErrorSound();
				HAL_Delay(3000);
				printMiadetBarati(0, 2);
				break;
			case 299:
				//takeData(buffer, count, dispData);
				HD44780_Clear();
				HD44780_SetCursor(0, 0);
				HD44780_PrintStr((char*) dispData);
				ErrorSound();
				HAL_Delay(3000);
				printMiadetBarati(0, 2);
				break;
			default:
				memset(dispData, 0, sizeof(dispData));
				printMiadetBarati(0, 2);

	  }
		memset(BUFFER, '\0', sizeof BUFFER / sizeof BUFFER[0]);
	}
  /* USER CODE END process_status_task */
}

/* USER CODE BEGIN Header_send_card_data_MQTT */
/**
* @brief Function implementing the postenabled thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_send_card_data_MQTT */
void send_card_data_MQTT(void const * argument)
{
  /* USER CODE BEGIN send_card_data_MQTT */
  /* Infinite loop */


	//xSemaphoreTake(semaphore_to_do_post, portMAX_DELAY);
	for(;;)
	{
		xEventGroupWaitBits(status_event, request_make_request_allowed, pdTRUE, pdTRUE, portMAX_DELAY);

		xEventGroupSetBits(status_event, card_read_NOT_allowed);
		xEventGroupClearBits(status_event, card_read_allowed);
		//vSemaphoreDelete(semaphore_to_do_post);
		printDaicadet(0, 4);
		send_card_data(LENGTH, postData);

		uxHighWaterMark_for_post_mqtt = uxTaskGetStackHighWaterMark( NULL );

	}
  /* USER CODE END send_card_data_MQTT */
}

/* USER CODE BEGIN Header_check_MQTT */
/**
* @brief Function implementing the check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_check_MQTT */
void check_MQTT(void const * argument)
{
  /* USER CODE BEGIN check_MQTT */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END check_MQTT */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
