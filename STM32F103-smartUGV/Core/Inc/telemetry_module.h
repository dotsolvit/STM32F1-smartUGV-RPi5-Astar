/* telemetry_module.h */
#ifndef __TELEMETRY_MODULE_H
#define __TELEMETRY_MODULE_H

#include "main.h"

// Фізичні параметри UGV (реальні значення)
#define WHEEL_DIAMETER_M      0.066f  // Діаметр колеса 66 мм
#define ENCODER_PULSES_PER_REV 20.0f  // Кількість імпульсів енкодера на 1 повний оберт колеса
#define PI_F                  3.14159265f

// Розрахунковий коефіцієнт: скільки метрів робот проходить за 1 імпульс енкодера
#define METERS_PER_PULSE      ((PI_F * WHEEL_DIAMETER_M) / ENCODER_PULSES_PER_REV)

// Глобальні змінні модуля телеметрії
extern float robot_x;
extern float robot_y;
extern float robot_theta_deg;
extern uint8_t robot_battery_pct;

// Інтерфейсні функції
void Telemetry_Init(void);
void Telemetry_Update(void);

#endif
