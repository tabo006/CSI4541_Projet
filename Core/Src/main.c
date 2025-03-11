/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* Definitions for the Button Task */
osThreadId_t TaskButtonHandle;
const osThreadAttr_t TaskButton_attributes = {
  .name = "TaskButton",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for the LDR Task */
osThreadId_t TaskLDRHandle;
const osThreadAttr_t TaskLDR_attributes = {
  .name = "TaskLDR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for the Buzzer Task */
osThreadId_t TaskBuzzerHandle;
const osThreadAttr_t TaskBuzzer_attributes = {
  .name = "TaskBuzzer",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for the LED Task */
osThreadId_t TaskLedHandle;
const osThreadAttr_t TaskLed_attributes = {
  .name = "TaskLed",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for the Servo Task */
osThreadId_t TaskServoHandle;
const osThreadAttr_t TaskServo_attributes = {
  .name = "TaskServo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

void StartTaskButton(void *argument);
void StartTaskLDR(void *argument);
//laser
void StartTaskBuzzer(void *argument);
void StartTaskLed(void *argument);
void StartTaskServo(void *argument);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Uncomment the test you want to run */
  // test_LDR();
  // test_Laser();
  // test_Buzzer();
  // test_LED();
  // test_Servo();
  // test_ESP8266();
  // test_Button();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  TaskButtonHandle = osThreadNew(StartTaskButton, NULL, &TaskButton_attributes);
  TaskLDRHandle = osThreadNew(StartTaskLDR, NULL, &TaskLDR_attributes);
  //laser
  TaskBuzzerHandle = osThreadNew(StartTaskBuzzer, NULL, &TaskBuzzer_attributes);
  TaskLedHandle = osThreadNew(StartTaskLed, NULL, &TaskLed_attributes);
  TaskServoHandle = osThreadNew(StartTaskServo, NULL, &TaskServo_attributes);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
void StartTaskButton(void *argument) {
	for(;;)
	{
		// Check if PC13 button is pressed (Active LOW)
		if (HAL_GPIO_ReadPin(GPIOC, Stop_button_Pin) == GPIO_PIN_RESET) {
			HAL_Delay(50); // Debounce delay

			// Toggle LED (PA5)
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			// Wait for button release
			while (HAL_GPIO_ReadPin(GPIOC, Stop_button_Pin) == GPIO_PIN_RESET);
			HAL_Delay(50); // Prevent multiple triggers
		}
	}
}
/*Test LDR Sensor (PA0 - ADC1) */
void StartTaskLDR(void *argument) {
	for(;;)
	{
		uint32_t adcValue;
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adcValue = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);

		if (adcValue < 1000) {  // Adjust threshold as needed
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // Turn on LED if laser is broken
		} else {
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		HAL_Delay(500);
		}

}
/* Test Laser Pointer (PB1 - GPIO Output) */
void test_Laser() {
    HAL_GPIO_WritePin(GPIOB, Laser_Pin, GPIO_PIN_SET);  // Laser ON
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOB, Laser_Pin, GPIO_PIN_RESET); // Laser OFF
    HAL_Delay(2000);
}
/* Test Buzzer (PB0 - GPIO Output) */
void StartTaskBuzzer(void *argument) {
	for(;;)
	{
		HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_SET);  // Buzzer ON
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET); // Buzzer OFF
		HAL_Delay(1000);
	}
}
void StartTaskLed(void *argument) {
	for(;;){
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  // LED ON
		HAL_Delay(1000);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LED OFF
		HAL_Delay(1000);
	}
}
void StartTaskServo(void *argument) {
	for(;;)
	{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);  // Move to -90°
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500); // Move to 0° (neutral)
		HAL_Delay(4000);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2500); // Move to +90°
		HAL_Delay(2000);
	}
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
