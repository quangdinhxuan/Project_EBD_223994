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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include<stdarg.h>
#include<stdio.h>
#define UART1_ADDRESS_BASE 0x40011000
#define GPIOB_ADDRESS_BASE 0x40020400

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void uart_init()
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	//chon PB6,PB7
	uint32_t* MODER = (uint32_t*)(GPIOB_ADDRESS_BASE + 0x00);
	//PB6,PB7
	*MODER &= ~((0b11 << 12) |( 0b11 << 14));
	*MODER |= ((0b10 << 12) |( 0b10 << 14));
	//ALTERNATIVE FUNCTION
	uint32_t* AFRL = (uint32_t*)(GPIOB_ADDRESS_BASE + 0x20);
	//CLEAR PB6,PB7
	*AFRL &= ~((0xf << 24) | (0xf << 28));
	//AF07 NEN DICH 7
	*AFRL |= ((7 << 24) | (7 << 28));



	//baund-rate=9600
	// parity: even
	// data length: 8bit
	__HAL_RCC_USART1_CLK_ENABLE();
	uint32_t* BRR = (uint32_t*)(UART1_ADDRESS_BASE + 0x08);
	//104,166667=16000000/(9600*160)
	//3=0.166667*16 fraction part
	//Baud_rate 19.6.3
	*BRR |= (104 << 4) | (3 << 0);
	uint32_t* CR1 = (uint32_t*)(UART1_ADDRESS_BASE + 0x0c);
	//12 xac dinh length(8 or 9 bits co parity hay k)
	//10 xac dinh parity
	*CR1 |= (1 << 12) | (1 << 10);
	//9 chon even hay odd parity neu 0 thi co the ko can
	*CR1 &= ~(1 << 9);
	//3 transmit,2 receive,13 enable uart
//	*CR1 |= (1 << 5);//enable interrupt, rxne=1
//	uint32_t*ISER1=(uint32_t*)(0xe000e104) ;
//	*ISER1|=(1<<(37-32));
	uint32_t*CR3=(uint32_t*)(0x40011014);
	*CR3|=(1<<6);


	*CR1 |= (1 << 13) | (1 << 3) | (1 << 2);



}
char rx_buf[1024];
int rx_index=0;
void USART1_IRQHandler()
{
	uint32_t* SR = (uint32_t*)(UART1_ADDRESS_BASE + 0x00);
	//DR DATA REGISTER
		uint32_t* DR = (uint32_t*)(UART1_ADDRESS_BASE + 0x04);
		rx_buf[rx_index++]=*DR;
		*SR &=~(1<<5);
}
void uart_send_1_byte(char data)
{//SR STATUS REGISTER
	uint32_t* SR = (uint32_t*)(UART1_ADDRESS_BASE + 0x00);
//DR DATA REGISTER
	uint32_t* DR = (uint32_t*)(UART1_ADDRESS_BASE + 0x04);
	//Xet transmit empty thi moi trans
	while (((*SR >> 7) & 1) != 1);
	*DR = data;
	//6 la transmit complete
	while (((*SR >> 6) & 1) != 1);

}
void uart_send_string(char* str)
{
	int str_len = strlen(str);
	for (int i = 0; i < str_len; i++) {
		uart_send_1_byte(str[i]);
	}

}
char uart_recv_data()
{
	uint32_t* SR = (uint32_t*)(UART1_ADDRESS_BASE + 0x00);
	uint32_t* DR = (uint32_t*)(UART1_ADDRESS_BASE + 0x04);
	while (((*SR >> 5) & 1) != 1);
	uint8_t data = *DR;
	return data;

}
void printlog(char*format,...)
{
char buf[1024]={0};
char buf_len=0;
va_list _ArgList;
va_start(_ArgList,format);
vsprintf(buf,format,_ArgList);
uart_send_string(buf);
va_end(_ArgList);
buf_len=strlen(buf);
for(int i=0;i<buf_len;i++)
{
	uart_send_1_byte(buf[i]);
}
}
void custom_printf(char*format,...)
{
char buf[1024]={0};
va_list _ArgList;
va_start(_ArgList,format);
vsprintf(buf,format,_ArgList);
uart_send_string(buf);
va_end(_ArgList);
}
int common_memory;
void led_init()
{
	//enable clock GPIOD
	__HAL_RCC_GPIOD_CLK_ENABLE();
	//Set PD12,13,14,15 in OUTPUT mode(push-pull)
	uint32_t* GPIOD_MODER=(uint32_t*)(0x40020C00);
	*GPIOD_MODER &= ~(0b11111111 << 24);
	*GPIOD_MODER |=(0b01 << 24)|(0b01 << 26)|(0b01<<28) | (0b01<<30);

}
typedef enum
{
	LED_GREEN=12,
	LED_ORANGE=13,
	LED_RED=14,
	LED_BLUE=15
}led_t;
/**
* @brief Control the LED
* @param:
* led: the led is controlled
* state: led state to controll
* @retval None
*/
void led_ctrl(uint8_t led,uint8_t state)
{
	//write state into output data register
	uint32_t* GPIOD_ODR =(uint32_t*)(0x40020C14);
	if(state==1){

	//set pit in led to 1
	*GPIOD_ODR |=(1<<led);
	}
	else
	{
		//clear pin in led to 0
		*GPIOD_ODR &=~(1<<led);
	}
}
void led_toggle(uint8_t led)
{
	uint32_t* GPIOD_ODR =(uint32_t*)(0x40020C14);
	uint32_t* GPIOD_IDR =(uint32_t*)(0x40020C10);
		if(((*GPIOD_IDR>>led)&1)==1){


		*GPIOD_ODR &=~(1<<led);
		}
		else
		{

			*GPIOD_ODR |=(1<<led);
		}
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for task01 */
osThreadId_t task01Handle;
const osThreadAttr_t task01_attributes = {
  .name = "task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task02 */
osThreadId_t task02Handle;
const osThreadAttr_t task02_attributes = {
  .name = "task02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task03 */
osThreadId_t task03Handle;
const osThreadAttr_t task03_attributes = {
  .name = "task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for task04 */
osThreadId_t task04Handle;
const osThreadAttr_t task04_attributes = {
  .name = "task04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensor_queue */
osMessageQueueId_t sensor_queueHandle;
const osMessageQueueAttr_t sensor_queue_attributes = {
  .name = "sensor_queue"
};
/* Definitions for uart_lock */
osMutexId_t uart_lockHandle;
const osMutexAttr_t uart_lock_attributes = {
  .name = "uart_lock"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void func1(void *argument);
void func2(void *argument);
void func3(void *argument);
void func4(void *argument);

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
  /* USER CODE BEGIN 2 */
uart_init();
printlog("xin chao:%f \r\n",3.14);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uart_lock */
  uart_lockHandle = osMutexNew(&uart_lock_attributes);

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
  /* creation of sensor_queue */
  sensor_queueHandle = osMessageQueueNew (16, sizeof(float), &sensor_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task01 */
  task01Handle = osThreadNew(func1, NULL, &task01_attributes);

  /* creation of task02 */
  task02Handle = osThreadNew(func2, NULL, &task02_attributes);

  /* creation of task03 */
  task03Handle = osThreadNew(func3, NULL, &task03_attributes);

  /* creation of task04 */
  task04Handle = osThreadNew(func4, NULL, &task04_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_func1 */
/**
  * @brief  Function implementing the task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_func1 */
void func1(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	led_toggle(LED_GREEN);
    osDelay(1000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_func2 */
/**
* @brief Function implementing the task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func2 */
void func2(void *argument)
{
  /* USER CODE BEGIN func2 */
  /* Infinite loop */
  for(;;)
  { int temp=0;
  osMessageQueueGet(sensor_queueHandle, &temp, 0, HAL_MAX_DELAY);
  custom_printf("hello world:%d\r\n",temp);
	     osDelay(1500);

  }
  /* USER CODE END func2 */
}

/* USER CODE BEGIN Header_func3 */
/**
* @brief Function implementing the task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func3 */
void func3(void *argument)
{
  /* USER CODE BEGIN func3 */
	  int sensor_value=0;
  /* Infinite loop */
  for(;;)
  {
	sensor_value++;
	osMessageQueuePut(sensor_queueHandle, &sensor_value, 0,HAL_MAX_DELAY);
    osDelay(1000);
  }
  /* USER CODE END func3 */
}

/* USER CODE BEGIN Header_func4 */
/**
* @brief Function implementing the task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_func4 */
void func4(void *argument)
{
  /* USER CODE BEGIN func4 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END func4 */
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
