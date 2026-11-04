/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  #define RX_STATE_WAIT_AA        0
  #define RX_STATE_WAIT_TYPE      1
  #define RX_STATE_READ_CONTROL   2
  #define RX_STATE_READ_PATH      3
  #define RX_STATE_WAIT_CRC       4

  if ((USART2->SR & USART_SR_RXNE) != 0) {
      uint8_t byte = (uint8_t)(USART2->DR & 0xFF);

      static uint8_t rx_state = RX_STATE_WAIT_AA;
      static uint8_t current_packet_type = 0;
      static uint8_t data_counter = 0;

      // Тимчасові буфери для приймання
      static ControlPacket_t temp_control;
      static PathPointPacket_t temp_path_pkt;

      switch (rx_state) {
          case RX_STATE_WAIT_AA:
              if (byte == 0xAA) {
                  rx_state = RX_STATE_WAIT_TYPE;
              }
              break;

          case RX_STATE_WAIT_TYPE:
              current_packet_type = byte;
              data_counter = 0;
              if (byte == 0xB1) {
                  rx_state = RX_STATE_READ_CONTROL;
              } else if (byte == 0xB2) {
                  rx_state = RX_STATE_READ_PATH;
              } else {
                  rx_state = RX_STATE_WAIT_AA; // Невідомий тип, скидаємо
              }
              break;

          case RX_STATE_READ_CONTROL:
              ((uint8_t*)&temp_control)[data_counter++] = byte;
              if (data_counter >= sizeof(ControlPacket_t)) {
                  rx_state = RX_STATE_WAIT_CRC;
              }
              break;

          case RX_STATE_READ_PATH:
              ((uint8_t*)&temp_path_pkt)[data_counter++] = byte;
              if (data_counter >= sizeof(PathPointPacket_t)) {
                  rx_state = RX_STATE_WAIT_CRC;
              }
              break;

          case RX_STATE_WAIT_CRC:
              if (current_packet_type == 0xB1) {
                  // Обробка команди Control (1 байт даних)
                  if (STM32_GetCRC8((uint8_t*)&temp_control.action, 1) == byte) {
                      if (temp_control.action == 1)      current_robot_state = ROBOT_STATE_MOVING;
                      else if (temp_control.action == 2) current_robot_state = ROBOT_STATE_STOPPED;
                      else if (temp_control.action == 3) current_robot_state = ROBOT_STATE_CALIBRATING;
                  }
              }
              else if (current_packet_type == 0xB2) {
                  // Обробка точки маршруту Path (9 байт даних: x(4) + y(4) + is_last(1))
                  if (STM32_GetCRC8((uint8_t*)&temp_path_pkt, sizeof(PathPointPacket_t)) == byte) {

                      // Якщо це найперша точка нового потоку — скидаємо старий лічильник безпечно
                      if (robot_new_path_ready == 1) {
                          robot_new_path_ready = 0;
                          robot_path_size = 0;
                      }

                      // Записуємо точку в масив, якщо є вільне місце
                      if (robot_path_size < MAX_PATH_POINTS) {
                          robot_path[robot_path_size].x = temp_path_pkt.x;
                          robot_path[robot_path_size].y = temp_path_pkt.y;
                          robot_path_size++;

                          // Якщо ESP32 каже, що це фінальна точка масиву
                          if (temp_path_pkt.is_last == 1) {
                              robot_new_path_ready = 1; // Сигнал для main.c: маршрут готовий!
                          }
                      }
                  }
              }
              rx_state = RX_STATE_WAIT_AA;
              break;

          default:
              rx_state = RX_STATE_WAIT_AA;
              break;
      }
  }

  if ((USART2->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) != 0) {
      volatile uint32_t dummy = USART2->DR;
      (void)dummy;
  }
  return;
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
