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

/* USER CODE BEGIN PV */
// Виділяємо пам'ять під стан робота. Початковий стан — СТОП.
volatile RobotState_t current_robot_state = ROBOT_STATE_STOPPED;

// Виділяємо пам'ять під маршрут робота.
volatile PathPoint_t robot_path[MAX_PATH_POINTS];
volatile uint8_t robot_path_size = 0;
volatile uint8_t robot_new_path_ready = 0;

//Організація асинхронного таймера відправки (50 мс)
uint32_t last_telemetry_tick = 0;
const uint32_t TELEMETRY_INTERVAL_MS = 50;

// Глобальні змінні стану робота (для демонстрації інтеграції)
float robot_x = 4.0f;
float robot_y = 4.0f;
float robot_theta_deg = 90.0f;
uint8_t robot_battery_pct = 80;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Асинхронна відправка телеметрії (викликається по таймеру кожні 50 мс)
void STM32_SendTelemetry(float x, float y, float theta, uint8_t battery, RobotState_t state) {
    UartTelemetryPacket_t pkt;
    pkt.header1 = 0xDE;
    pkt.header2 = 0x01;
    pkt.x = x;
    pkt.y = y;
    pkt.theta_deg = theta;
    pkt.battery_pct = battery;
    pkt.status_code = (uint8_t)state;

    // Рахуємо від першого корисного поля (x) до самого кінця структури
    uint16_t data_size = sizeof(UartTelemetryPacket_t) - 3;
    pkt.crc8 = STM32_GetCRC8((uint8_t*)&pkt.x, data_size);

    // Побайтова швидка відправка в регістр USART2
    uint8_t *pData = (uint8_t*)&pkt;
    for (uint8_t i = 0; i < sizeof(UartTelemetryPacket_t); i++) {
        while ((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = pData[i];
    }
}

// Асинхронна відправка однієї точки сканування перешкод
void STM32_SendScanPoint(int16_t angle, uint16_t distance_mm, uint8_t is_last) {
    UartScanPacket_t pkt;
    pkt.header1 = 0xDE;
    pkt.header2 = 0x02;
    pkt.angle = angle;
    pkt.distance_mm = distance_mm;
    pkt.is_end_of_scan = is_last;

    // Рахуємо від першого корисного поля (angle) до самого кінця структури
    uint16_t data_size = sizeof(UartScanPacket_t) - 3;
    pkt.crc8 = STM32_GetCRC8((uint8_t*)&pkt.angle, data_size);

    uint8_t *pData = (uint8_t*)&pkt;
    for (uint8_t i = 0; i < sizeof(UartScanPacket_t); i++) {
        while ((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = pData[i];
    }
}

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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Вмикаємо переривання по прийому байта
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint32_t current_tick = HAL_GetTick();

      // Асинхронний таймер відправки телеметрії кожні 50 мс
      if (current_tick - last_telemetry_tick >= TELEMETRY_INTERVAL_MS)
      {
          last_telemetry_tick = current_tick;

          // Виклик вашої функції відправки з актуальними даними
          STM32_SendTelemetry(robot_x, robot_y, robot_theta_deg, robot_battery_pct, current_robot_state);
      }

      // Логіка сканування перешкод (викликається покроково за логікою TIM1 сервоприводу)
      // Наприклад, коли серва стала в точку і Sharp зняв АЦП:
      // STM32_SendScanPoint(current_servo_angle, current_distance_mm, is_last_point);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
