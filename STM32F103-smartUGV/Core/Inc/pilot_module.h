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

// --- ПАРАМЕТРИ ЗУПИНКИ ТА ІНЕРЦІЇ ---
#define SLOWDOWN_DISTANCE_M     0.15f  // Дистанція початку зниження швидкості (15 см) //10
#define STOP_CONFIRM_TIME_MS    60     // Час без імпульсів енкодера для фіксації зупинки (60 мс)
#define STOP_TIMEOUT_MS         500    // Запобіжний таймаут очікування зупинки


// --- НАЛАШТУВАННЯ СЕРВОПРИВОДУ MG90S --- (TIM1_CH1, дискретність 1 мкс)
#define SERVO_MIN_PULSE        460   // +90 градусів (праворуч)  //450
#define SERVO_MAX_PULSE        2400  // -90 градусів (ліворуч) //2390

// --- ГЕОМЕТРІЯ ТА НАВІГАЦІЯ ---
#define SERVO_OFFSET_MM         60.0f  // Виліт сервоприводу вперед від центру робота
#define OBSTACLE_STOP_DIST_MM   400    // Поріг екстреної зупинки перед перешкодою (40 см) //300
#define WAYPOINT_TOLERANCE_M    0.05f  // Радіус зарахування точки маршруту (5 см)
#define HEADING_TOLERANCE_DEG   2.0f   // Точність наведення на курс при повороті (±2°) //4.0f

// --- ПАРАМЕТРИ СКАНУВАННЯ ---
#define NARROW_SCAN_MAX_DEG     11     // Сектор вузького огляду (±11°)
#define NARROW_SCAN_DELAY_MS    135    // Перевірений час стабілізації Sharp + серво
#define WIDE_SCAN_STEP_DEG      6      // Крок панорами (-90..+90)

// Напрямки руху
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD
} MotorDirection_t;

typedef enum {
    NAV_IDLE,
    NAV_TURNING,
    NAV_DRIVING,
    NAV_COASTING,       // Рух по інерції до повної зупинки коліс
    NAV_WIDE_SCANNING,
	NAV_CALIBRATING_RAMP  // Плавний розгін перед калібруванням компаса
} NavState_t;

typedef enum {
    STOP_REASON_NONE,
    STOP_REASON_WAYPOINT, // Досягнуто проміжну точку -> далі поворот
    STOP_REASON_ARRIVED,  // Досягнуто фініш маршруту -> статус ARRIVED
    STOP_REASON_OBSTACLE, // Помічено перешкоду -> далі широке сканування
    STOP_REASON_USER_CMD  // Команда STOP з сервера -> статус STOPPED
} StopReason_t;

extern volatile MotorDirection_t dir_left;
extern volatile MotorDirection_t dir_right;

// Інтерфейсні функції модуля
void Pilot_Init(void);
void Pilot_Update(void);
void Pilot_SetTargetSpeed(int16_t left, int16_t right);
void Pilot_StopImmediate(void);
void Pilot_SetServoAngle(int16_t angle_deg);
uint16_t Pilot_GetSharpDistanceMM(void);
#endif
