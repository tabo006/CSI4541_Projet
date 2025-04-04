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
#include "stdio.h"
#include "ssd1306.h"
#include "fonts.h"
#include "esp8266.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALARM_OFF 0
#define ALARM_ON 1
#define SYSTEM_OFF 0
#define SYSTEM_ON 1
#define DOOR_OPEN 0
#define DOOR_CLOSED 1

#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOC
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
int alarm_state = ALARM_OFF;
int light_value = -1;
volatile int system_state= SYSTEM_OFF;
int door_state= DOOR_OPEN;


uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

/* Definitions for the Button Task */
osThreadId_t TaskButtonHandle;
const osThreadAttr_t TaskButton_attributes = {
.name = "TaskButton",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t TaskSystemButtonHandle;
const osThreadAttr_t TaskSystemButton_attributes = {
.name = "TaskSystemButton",
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
osThreadId_t TaskLedSystemHandle;
const osThreadAttr_t StartTaskSystemLed_attributes = {
.name = "TaskSystemLed",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t TaskLaserHandle;
const osThreadAttr_t StartTaskLaser_attributes = {
.name = "TaskLaser",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t TaskUpdateWebHandle;
const osThreadAttr_t StartTaskUpdateWeb_attributes = {
.name = "TaskUpdateWeb",
.stack_size = 1028 * 4,
.priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t TaskDistanceHandle;
const osThreadAttr_t StartTaskDistance_attributes = {
.name = "TaskDistance",
.stack_size = 1028 * 4,
.priority = (osPriority_t) 25,
};
osMutexId_t systemStateMutex;
const osMutexAttr_t systemStateMutex_attributes = {
   .name = "systemStateMutex"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
void delay (uint16_t time);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HCSR04_Read (void);
void StartTaskButton(void *argument);
void StartTaskSystemButton(void *argument);
void StartTaskLDR(void *argument);
void StartTaskUpdateWeb(void *argument);
//laser
void StartTaskBuzzer(void *argument);
void StartTaskLed(void *argument);
void StartTaskServo(void *argument);
void StartTaskSystemLed(void *argument);
void StartTaskLaser(void *argument);
void StartTaskDistance(void *argument);
const char *intrusionHTML = "<p style='color:red;'>Intrusion Detected</p>";
const char *regularHTML = "<p style='color:blue;'>System Armed no intrusion</p>";
const char *systemoffHTML = "<p style='color:blue;'>System Disarmed</p>";
const char *localIP =    "172.20.10.2"; //change this to yours
const char *pwd = "chloe1908";
const char *username = "Chloe";

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
* @brief  The application entry point.
* @retval int
*/

void delay (uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER(&htim3) < time);
}

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
/* USER CODE BEGIN 2 */
SSD1306_Init(); // Initialize OLED display
SSD1306_Clear(); // Clear the display
SSD1306_GotoXY(0, 10); // Set cursor position to (0, 0)
SSD1306_Puts("main", &Font_11x18, 1); // Display simple message
SSD1306_UpdateScreen(); // Update the screen
/* Uncomment the test you want to run */
// test_LDR();
// test_Laser();
// test_Buzzer();
// test_LED();
// test_Servo();
// test_ESP8266();
HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
ESP_Server_Init(username, pwd);

sendHTMLToLocalServer(localIP, regularHTML);
systemStateMutex = osMutexNew(&systemStateMutex_attributes);
if (systemStateMutex == NULL) {
    printf("Failed to create system state mutex\n");
}
// test_Button();
/* USER CODE END 2 */
/* Init scheduler */
osKernelInitialize();
/* Call init function for freertos objects (in freertos.c) */
MX_FREERTOS_Init();
TaskButtonHandle = osThreadNew(StartTaskButton, NULL, &TaskButton_attributes);
TaskSystemButtonHandle = osThreadNew(StartTaskSystemButton, NULL, &TaskSystemButton_attributes);
TaskLDRHandle = osThreadNew(StartTaskLDR, NULL, &TaskLDR_attributes);
//laser
TaskBuzzerHandle = osThreadNew(StartTaskBuzzer, NULL, &TaskBuzzer_attributes);
TaskLedHandle = osThreadNew(StartTaskLed, NULL, &TaskLed_attributes);
TaskServoHandle = osThreadNew(StartTaskServo, NULL, &TaskServo_attributes);
TaskLedSystemHandle = osThreadNew(StartTaskSystemLed, NULL, &StartTaskSystemLed_attributes);
TaskLaserHandle = osThreadNew(StartTaskLaser, NULL, &StartTaskLaser_attributes);
TaskUpdateWebHandle = osThreadNew(StartTaskUpdateWeb, NULL, &StartTaskUpdateWeb_attributes);
TaskDistanceHandle = osThreadNew(StartTaskDistance, NULL, &StartTaskDistance_attributes);
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
		if (HAL_GPIO_ReadPin(GPIOC, B1_Pin) == GPIO_PIN_RESET && system_state == SYSTEM_ON) {
			HAL_Delay(50); // Debounce delay
			// Toggle LED (PA5)
			alarm_state = ALARM_OFF;
			// Wait for button release
			while (HAL_GPIO_ReadPin(GPIOC, B1_Pin) == GPIO_PIN_RESET);
			HAL_Delay(50); // Prevent multiple triggers
		}
	}
}

void StartTaskDistance(void *argument) {
	for(;;)
	{
		 HCSR04_Read();  // Trigger distance measurement
		 HAL_Delay(200);  // Delay to allow sensor to respond

		 char buffer[10];
		 SSD1306_Clear();
		 SSD1306_GotoXY(0, 10);
		 SSD1306_Puts("Distance", &Font_11x18, 1);
		 SSD1306_GotoXY(0, 30);
		 sprintf(buffer, "%d cm", Distance);
		 SSD1306_Puts(buffer, &Font_11x18, 1);
		 SSD1306_UpdateScreen();
		 printf(Distance);
		 if(system_state == SYSTEM_ON && Distance < 5){
			 alarm_state = ALARM_ON;
		 }

		 if(alarm_state == ALARM_OFF){
			 osDelay(2000);
		 }
		 osDelay(500);  // Delay to avoid excessive updates
		}
	}


void StartTaskUpdateWeb(void *argument) {
    int prev_alarm_state = ALARM_OFF;  // Track previous state && prev_alarm_state == ALARM_OFF
    int prev_system_state = SYSTEM_ON;

    for (;;) {
    	printf("Current Alarm State: %d | Previous Alarm State: %d\n", alarm_state, prev_alarm_state);

        if (alarm_state == ALARM_ON && prev_alarm_state == ALARM_OFF) {
            // Alarm just turned ON
            sendHTMLToLocalServer(localIP, intrusionHTML);
        }
        else if (alarm_state == ALARM_OFF && prev_alarm_state == ALARM_ON && system_state== SYSTEM_OFF) {
            // Alarm just turned OFF
            sendHTMLToLocalServer(localIP, systemoffHTML);
        }else if (alarm_state == ALARM_OFF && prev_system_state== SYSTEM_ON && system_state== SYSTEM_OFF) {
            // Alarm just turned OFF
            sendHTMLToLocalServer(localIP, systemoffHTML);
        }else if (alarm_state == ALARM_OFF && prev_alarm_state == ALARM_ON){
        	sendHTMLToLocalServer(localIP, regularHTML);
        }else if (alarm_state == ALARM_OFF && prev_system_state == SYSTEM_OFF && system_state== SYSTEM_ON){
        	sendHTMLToLocalServer(localIP, regularHTML);
        }

        // Update previous state
        prev_alarm_state = alarm_state;
        prev_system_state = system_state;

        osDelay(1000);  // Delay before checking again
    }
}

void StartTaskSystemButton(void *argument) {
   for (;;) {
       if (HAL_GPIO_ReadPin(GPIOC, control_System_Pin) == GPIO_PIN_RESET) {
           HAL_Delay(50); // Debounce delay
           osMutexAcquire(systemStateMutex, osWaitForever); // Lock mutex
           system_state = !system_state;
           if (system_state == SYSTEM_OFF){
               alarm_state = ALARM_OFF;
           }
           osMutexRelease(systemStateMutex); // Release mutex
           // Wait for button release
           while (HAL_GPIO_ReadPin(GPIOC, control_System_Pin) == GPIO_PIN_RESET);
           HAL_Delay(50); // Prevent multiple triggers
       }
   }
}
void StartTaskLDR(void *argument) {
   int local_system_state = SYSTEM_OFF;  // Track the last known state
   for (;;) {
       // Acquire mutex before reading system_state
       osMutexAcquire(systemStateMutex, osWaitForever);
       int current_system_state = system_state;
       osMutexRelease(systemStateMutex);
       // Check if system just turned ON
       if (current_system_state == SYSTEM_ON && local_system_state == SYSTEM_OFF) {
           osDelay(2000);  // Delay before first LDR reading
       }
       // Update local state
       local_system_state = current_system_state;
       if (current_system_state == SYSTEM_ON) {
    	   if(alarm_state == ALARM_OFF){
    	  			 osDelay(200);
    	  		 }
           uint32_t adcValue;
           HAL_ADC_Start(&hadc1);
           HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
           adcValue = HAL_ADC_GetValue(&hadc1);
           HAL_ADC_Stop(&hadc1);
           light_value = adcValue;
           if (adcValue < 1500 && system_state == SYSTEM_ON) {
               alarm_state = ALARM_ON;

           }
       }
       osDelay(500);  // Regular delay between reads
   }
}


void StartTaskLaser(void *argument) {
   for(;;) {
       if (system_state == SYSTEM_ON) {
           HAL_GPIO_WritePin(GPIOB, Laser_Pin, GPIO_PIN_RESET);
       } else {
           HAL_GPIO_WritePin(GPIOB, Laser_Pin, GPIO_PIN_SET);
       }
       osDelay(100);
   }
}
/* Test Buzzer (PB0 - GPIO Output) */
void StartTaskBuzzer(void *argument) {
	for(;;)
	{
		if(alarm_state == ALARM_ON){
			HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_SET);  // Buzzer ON
			HAL_Delay(2000);
			HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET); // Buzzer OFF
			HAL_Delay(1000);
		}
// Buzzer ON
	}
}
void StartTaskLed(void *argument) {
	for(;;){
		if(alarm_state==ALARM_ON){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
		  // LED ON
	}
}
void StartTaskSystemLed(void *argument) {
   for(;;) {
       if (system_state == SYSTEM_ON) {
           HAL_GPIO_WritePin(GPIOA, System_state_Pin, GPIO_PIN_SET);
       } else {
           HAL_GPIO_WritePin(GPIOA, System_state_Pin, GPIO_PIN_RESET);
       }
       osDelay(100);
   }
}
void StartTaskServo(void *argument) {
	for(;;)
	{

		osDelay(1000);
		 if(alarm_state == ALARM_ON && door_state == DOOR_OPEN) {
		            // Move both servos 90° to the left
		            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);  // 90° for first servo
		            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);  // 90° for second servo (same direction)
		            HAL_Delay(2000);
		            door_state = DOOR_CLOSED;
		        }
		        else if(alarm_state == ALARM_OFF && door_state == DOOR_CLOSED) {
		            // Move both servos back to 0° (neutral)
		            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);  // Back to 0° for first servo
		            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2500);   // Back to 0° for second servo (same direction)
		            HAL_Delay(2000);
		            door_state = DOOR_OPEN;
		        }
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // If the interrupt is from channel 1
        if (Is_First_Captured == 0) { // If it's the first edge (rising)
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read first value
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // Switch to falling edge
        } else if (Is_First_Captured == 1) { // If it's the second edge (falling)
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read second value
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // Reset to rising edge

            // Compute the difference
            if (IC_Val2 > IC_Val1) {
                Difference = IC_Val2 - IC_Val1;
            } else {
                Difference = (0xFFFF - IC_Val1) + IC_Val2;
            }

            Distance = Difference * 0.034 / 2; // Convert to cm

            Is_First_Captured = 0; // Reset flag for next measurement
        }
    }
}


void HCSR04_Read (void) {
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // Set TRIG pin HIGH
    delay(10);  // 10µs pulse
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // Set TRIG pin LOW

    // Reset first capture flag
    Is_First_Captured = 0;

    // Start the input capture interrupt
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
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
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
  return len;
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
