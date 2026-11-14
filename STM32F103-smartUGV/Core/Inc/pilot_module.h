/* pilot_module.h */
#ifndef __PILOT_MODULE_H
#define __PILOT_MODULE_H

#include "main.h"

// Константи прискорення (дискретність налаштування ШІМ на один крок циклу)
#define MOTOR_MAX_PWM          99    // Максимум відповідно до ARR=99
#define MOTOR_MIN_PWM          68    // Поріг, при якому мотори починають крутитися (Deadband)
#define MOTOR_ACCEL_STEP       2     // Крок плавного розгону за один цикл (20 мс)
#define MOTOR_DECEL_STEP       5     // Крок плавного гальмування (швидший за розгін)
#define DRIVE_BASE_SPEED        82    // Крейсерська швидкість на прямій
#define TURN_MAX_SPEED          78    // Швидкість розвороту на місці

// Налаштування сервоприводу MG90S (TIM1_CH1, дискретність 1 мкс)
#define SERVO_MIN_PULSE        460   // +90 градусів (праворуч)  //450
#define SERVO_MAX_PULSE        2400  // -90 градусів (ліворуч) //2390

// Параметри геометрії та навігації
#define SERVO_OFFSET_MM         60.0f // Виліт сервоприводу вперед від центру робота
#define SAFETY_MARGIN_MM        150.0f// Запас дистанції для безпечної зупинки перед перешкодою
#define WAYPOINT_TOLERANCE_M    0.08f // Радіус зарахування точки маршруту (8 см)
#define HEADING_TOLERANCE_DEG   4.0f  // Точність наведення на курс при повороті (±4°)

// Параметри сканування
#define NARROW_SCAN_MAX_DEG     11    // Сектор вузького огляду (±11°)
#define WIDE_SCAN_STEP_DEG      6     // Крок панорами (-90..+90)

// Напрямки руху
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD
} MotorDirection_t;

//Розкладемо рух по масиву robot_path на чіткі фази:
typedef enum {
    NAV_IDLE,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_OBSTACLE_STOPPING,
    NAV_WIDE_SCANNING
} NavState_t;

extern volatile MotorDirection_t dir_left;
extern volatile MotorDirection_t dir_right;

// Інтерфейсні функції модуля
void Pilot_Init(void);
void Pilot_Update(void);
void Pilot_SetTargetSpeed(int16_t left, int16_t right);

// Нові функції керування головою та сканером
void Pilot_SetServoAngle(int16_t angle_deg);
uint16_t Pilot_GetSharpDistanceMM(void);

#endif
