/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define MAX_PATH_POINTS 50 // Максимальна кількість точок у маршруті із запасом

// Визначення станів головного автомата машинки
typedef enum {
    ROBOT_STATE_STOPPED = 0,
    ROBOT_STATE_MOVING  = 1,
    ROBOT_STATE_CALIBRATING = 2
} RobotState_t;

// Структура однієї точки маршруту (8 байт)
typedef struct {
    float x;
    float y;
} PathPoint_t;

// Вирівнювання структури по 1 байту
#pragma pack(push, 1)
typedef struct {
    uint8_t action;         // 1 - start, 2 - stop, 3 - calibrate
} ControlPacket_t;
#pragma pack(pop)

// Вирівнювання структури пакета маршрута по 1 байту, як в ESP32 (10 байт даних під CRC)
#pragma pack(push, 1)
typedef struct {
    float x;
    float y;
    uint8_t is_last;
} PathPointPacket_t;
#pragma pack(pop)

#pragma pack(push, 1)
// Пакет для відправки 1: Телеметрія для robot/telemetry
typedef struct {
    uint8_t header1;       // 0xDE
    uint8_t header2;       // 0x01
    uint8_t crc8;          // Контрольна сума всього, що йде ДАЛІ
    // --- Корисні дані (можна міняти, додавати, видаляти поля) ---
    float x;
    float y;
    float theta_deg;
    uint8_t battery_pct;
    uint8_t status_code; // 0-stopped, 1-moving, 2-scanning, 3-calibrating, 4-arrived
} UartTelemetryPacket_t;

// Пакет для відправки 2: Одиничний вимір для robot/obstacle_scan
typedef struct {
    uint8_t header1;       // 0xDE
    uint8_t header2;       // 0x02
    uint8_t crc8;          // Контрольна сума всього, що йде ДАЛІ
    // --- Корисні дані ---
    int16_t angle;      // Поточний кут голови (-90...90)
    uint16_t distance_mm;  // Відстань в мм від Sharp
    uint8_t is_end_of_scan; // 1 якщо це остання точка панорами, інакше 0
} UartScanPacket_t;
#pragma pack(pop)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Оголошуємо, що змінна існує, щоб її бачило переривання в stm32f1xx_it.c
extern volatile RobotState_t current_robot_state;

// Наш глобальний маршрут для алгоритму наведення
extern volatile PathPoint_t robot_path[MAX_PATH_POINTS];
extern volatile uint8_t robot_path_size;       // Фактична кількість прийнятих точок
extern volatile uint8_t robot_new_path_ready;  // 1 - прийшов новий повний маршрут, 0 - оброблено
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
// Загальна швидка функція обчислення CRC8 Dallas/Maxim для всього проєкту
static inline uint8_t STM32_GetCRC8(uint8_t *buffer, uint8_t size) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < size; i++) {
        uint8_t data = buffer[i];
        for (uint8_t j = 8; j > 0; j--) {
            uint8_t mix = (crc ^ data) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            data >>= 1;
        }
    }
    return crc;
}
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
