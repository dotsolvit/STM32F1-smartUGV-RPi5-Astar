#ifndef __HP5883_H
#define __HP5883_H

#include "stm32f1xx_hal.h"

// Фізична I2C адреса чіпа HP5883 / QMC5883P
#define HP5883_I2C_ADDR         (0x2C << 1)

// Структура для збереження відкаліброваних коефіцієнтів та поточних даних
typedef struct {
    I2C_HandleTypeDef *hi2c; // Вказівник на апаратний I2C HAL
    int16_t x_offset;        // Зсув Hard Iron по осі X
    int16_t y_offset;        // Зсув Hard Iron по осі Y
    int16_t mag_x;           // Останні сирі дані X
    int16_t mag_y;           // Останні сирі дані Y
    int16_t mag_z;           // Останні сирі дані Z
    float heading;           // Розрахований відкалібрований кут (0.0 - 360.0°)
} HP5883_HandleTypeDef;

// Офіційні прототипи трьох функцій
HAL_StatusTypeDef HP5883_Init(HP5883_HandleTypeDef *g_compass, I2C_HandleTypeDef *hi2c_bus);
void               HP5883_RunCalibration(HP5883_HandleTypeDef *g_compass, uint32_t duration_ms);
HAL_StatusTypeDef HP5883_Read(HP5883_HandleTypeDef *g_compass);

#endif /* __HP5883_H */
