#include "hp5883.h"
#include <math.h>

// Локальні регістри пристрою
#define REG_DATA_X   0x01
#define REG_STATUS   0x09
#define REG_CTRL1    0x0A
#define REG_CTRL2    0x0B

HAL_StatusTypeDef HP5883_Init(HP5883_HandleTypeDef *g_compass, I2C_HandleTypeDef *hi2c_bus) {
    uint8_t data;
    g_compass->hi2c = hi2c_bus;
    g_compass->heading = 0.0f;

    // 1. Програмне скидання
    data = 0x80;
    if (HAL_I2C_Mem_Write(g_compass->hi2c, HP5883_I2C_ADDR, REG_CTRL2, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(50);

    // 2. Вмикаємо Set/Reset режим + Діапазон 8 Gauss
    data = 0x08;
    HAL_I2C_Mem_Write(g_compass->hi2c, HP5883_I2C_ADDR, REG_CTRL2, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    // 3. Запуск безперервного режиму (Continuous Mode, макс фільтрація CF=200Hz, C3=10Hz)
    data = 0xC3;  //0x0D;
    if (HAL_I2C_Mem_Write(g_compass->hi2c, HP5883_I2C_ADDR, REG_CTRL1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HP5883_RunCalibration(HP5883_HandleTypeDef *g_compass, uint32_t duration_ms) {
    int16_t x_min = 32767, x_max = -32768;
    int16_t y_min = 32767, y_max = -32768;
    uint8_t status_reg = 0;
    uint8_t buf[6];
    uint32_t start_time = HAL_GetTick();

    // Блокуючий цикл ТІЛЬКИ на час калібрування (поки користувач крутить машинку)
    while ((HAL_GetTick() - start_time) < duration_ms) {
        HAL_I2C_Mem_Read(g_compass->hi2c, HP5883_I2C_ADDR, REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, 10);

        if (status_reg & 0x01) {  //Check status register 09H[0] ,”1” means ready
            if (HAL_I2C_Mem_Read(g_compass->hi2c, HP5883_I2C_ADDR, REG_DATA_X, I2C_MEMADD_SIZE_8BIT, buf, 6, 10) == HAL_OK) {
                int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
                int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);

                if (raw_x < x_min) x_min = raw_x;
                if (raw_x > x_max) x_max = raw_x;
                if (raw_y < y_min) y_min = raw_y;
                if (raw_y > y_max) y_max = raw_y;
            }
        }
        HAL_Delay(5); // Не перевантажуємо шину в циклі
    }

    // Записуємо фінальні коефіцієнти в структуру девайсу
    g_compass->x_offset = (x_max + x_min) / 2;
    g_compass->y_offset = (y_max + y_min) / 2;
}

HAL_StatusTypeDef HP5883_Read(HP5883_HandleTypeDef *g_compass) {
    uint8_t status_reg = 0;
    uint8_t buf[6];

    HAL_I2C_Mem_Read(g_compass->hi2c, HP5883_I2C_ADDR, REG_STATUS, I2C_MEMADD_SIZE_8BIT, &status_reg, 1, 10);

    if (status_reg & 0x01) {  //Check status register 09H[0] ,”1” means ready
        if (HAL_I2C_Mem_Read(g_compass->hi2c, HP5883_I2C_ADDR, REG_DATA_X, I2C_MEMADD_SIZE_8BIT, buf, 6, 10) == HAL_OK) {
            g_compass->mag_x = (int16_t)((buf[1] << 8) | buf[0]);
            g_compass->mag_y = (int16_t)((buf[3] << 8) | buf[2]);
            g_compass->mag_z = (int16_t)((buf[5] << 8) | buf[4]);

            // Математика з урахуванням збережених коефіцієнтів
            float cal_x = (float)(g_compass->mag_x - g_compass->x_offset);
            float cal_y = (float)(g_compass->mag_y - g_compass->y_offset);

            g_compass->heading = atan2(cal_y, cal_x) * 57.29578f;
            if (g_compass->heading < 0) {
                g_compass->heading += 360.0f;
            }
            return HAL_OK;
        }
    }
    return HAL_BUSY; // Повертаємо BUSY, якщо дані ще не оновилися
}
