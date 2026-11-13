/* pilot_module.h */
#ifndef __PILOT_MODULE_H
#define __PILOT_MODULE_H

#include "main.h"

// Константи прискорення (дискретність налаштування ШІМ на один крок циклу)
#define MOTOR_MAX_PWM          99    // Максимум відповідно до ARR=99
#define MOTOR_MIN_PWM          68    // Поріг, при якому мотори починають крутитися (Deadband)
#define MOTOR_ACCEL_STEP       2     // Крок плавного розгону за один цикл (20 мс)
#define MOTOR_DECEL_STEP       5     // Крок плавного гальмування (швидший за розгін)

// Напрямки руху
typedef enum {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_TURN_LEFT,  // Розворот протиходом на місці
    DIR_TURN_RIGHT  // Розворот протиходом на місці
} MotorDirection_t;

extern volatile MotorDirection_t dir_left;
extern volatile MotorDirection_t dir_right;

// Інтерфейсні функції модуля
void Pilot_Init(void);
void Pilot_Update(void);
void Pilot_SetTargetSpeed(int16_t left, int16_t right);

#endif
