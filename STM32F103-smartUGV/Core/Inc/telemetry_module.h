/* telemetry_module.h */
#ifndef __TELEMETRY_MODULE_H
#define __TELEMETRY_MODULE_H

#include "main.h"
#include "mpu6050.h"   // Драйвер MPU6050
#include "hp5883.h"    // Драйвер HP5883
#include "adc.h"       //ADC

// Фізичні параметри UGV (реальні значення)
#define WHEEL_DIAMETER_M      0.066f  // Діаметр колеса 66 мм
#define ENCODER_PULSES_PER_REV 20.0f  // Кількість імпульсів енкодера на 1 повний оберт колеса
#define PI_F                  3.14159265f

// Розрахунковий коефіцієнт: скільки метрів робот проходить за 1 імпульс енкодера
#define METERS_PER_PULSE      ((PI_F * WHEEL_DIAMETER_M) / ENCODER_PULSES_PER_REV)

// Константи резистивного дільника напруги (підставте ваші номінали, якщо відрізняються)
// Коефіцієнт дільника = (R_up + R_down) / R_down. 3.119 =(9.81k + 4.63k) / 4.63k voltage divider
#define BATT_DIVIDER_RATIO     3.119f

// Межі напруги LiPo батареї (Для 2S акумулятора))
#define BATT_MIN_VOLTAGE       6.8f   // 0% (по 3.4В на банку)
#define BATT_MAX_VOLTAGE       8.4f   // 100% (по 4.2В на банку)

#define FLASH_COMPASS_ADDR  0x0800FC00 // Робоча сторінка 63 для збережання даних компаса

// Глобальні змінні модуля телеметрії
extern float robot_x;
extern float robot_y;
extern float robot_theta_deg;
extern uint8_t robot_battery_pct;

//extern volatile uint16_t sharp_distance_raw_adc;

//Структура для збереження відкаліброваних коефіцієнтів та поточних даних HP5883
extern HP5883_HandleTypeDef compass;

// Створюємо екземпляр драйвера MPU6050
extern MPU6050_HandleTypeDef hmpu1;

// Інтерфейсні функції
void Telemetry_Init(void);
void Telemetry_Update(void);

//Калібрування компасу
void Telemetry_Calibration(void);

#endif
