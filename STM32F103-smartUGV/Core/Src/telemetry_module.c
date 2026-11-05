/* telemetry_module.c */
#include "telemetry_module.h"
#include "mpu6050.h"   // Драйвер MPU6050
#include "hp5883.h"    // Драйвер HP5883

// Експортуємо структури HAL та MPU з main.c
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

//Структура для збереження відкаліброваних коефіцієнтів та поточних даних HP5883
HP5883_HandleTypeDef compass;

// Створюємо екземпляр драйвера MPU6050
MPU6050_HandleTypeDef hmpu1;

// Оголошення глобальних координат
extern float robot_x;
extern float robot_y;
extern float robot_theta_deg;
extern uint8_t robot_battery_pct;

// Змінні для збереження попередніх значень лічильників
static uint16_t last_encoder_ticks = 0;
static uint32_t last_mpu_tick = 0;
static MPU_State_t prev_mpu_state = MPU_STATE_UNINIT;

// Розрахунок battery_pct через АЦП (ADC1)
uint8_t STM32_ReadBatteryPercentage(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

        // Приклад для опорної напруги 3.3В, 12-біт АЦП (4095) та дільника напруги
        float v_pin = ((float)adc_val * 3.3f) / 4095.0f;
        float v_batt = v_pin * 3.119f; //  3.119 =(9.81k + 4.63k) / 4.63k voltage divider

        // Мапування напруги LiPo в 0-100% (для 2S: 6.8V = 0%, 8.4V = 100%)
        float min_v = 6.8f;
        float max_v = 8.4f;

        if (v_batt <= min_v) return 0;
        if (v_batt >= max_v) return 100;

        return (uint8_t)(((v_batt - min_v) / (max_v - min_v)) * 100.0f);
    }
    return 0;
}

void Telemetry_Init(void) {
    // 1. Стартуємо апаратний лічильник TIM3 для енкодера
    HAL_TIM_Base_Start(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    last_encoder_ticks = 0;

    // 2. Стартуємо системний мікросекундний хронометр TIM2
    HAL_TIM_Base_Start(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // Оновлюємо початкові таймінги
    last_mpu_tick = HAL_GetTick();
    prev_mpu_state = hmpu1.state;

    // 3. Ініціалізація драйверів сенсорів
    //MPU6050
    // Ініціалізуємо драйвер, передаючи I2C1 та TIM2
    MPU6050_Init_Async(&hmpu1, &hi2c2, &htim2);
    // Крутимо автомат станів, поки датчик не перейде в режим READY
    while (hmpu1.state != MPU_STATE_READY && hmpu1.state != MPU_STATE_ERROR) {
        MPU6050_Process_State(&hmpu1);
    }
    // Беремо 500 точок для точного прорахунку нуля
    MPU6050_Calibrate_Gyro_Z(&hmpu1, 500);

    //HP5883
    HP5883_Init(&compass, &hi2c2);

    //Завжди починаємо з координат х=4.0м y=4.0м
    robot_x = 4.0f;
    robot_y = 4.0f;
    robot_theta_deg = 0.0f;
}

void Telemetry_Update(void) {
    // =================================================================
    // 1. Модуль Гіроскопа: Обробка станів та читання MPU6050 (Частота 50 Гц)
    // =================================================================

    // Обов'язково викликаємо Process_State для обслуговування I2C конекшну
    MPU6050_Process_State(&hmpu1);

    // Неблокуючий запуск читання кожні 20 мс
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - last_mpu_tick >= 20) {
        last_mpu_tick = current_tick;

        if (hmpu1.state == MPU_STATE_READY) {
            hmpu1.state = MPU_STATE_READING_DATA; // Переводимо в стан читання
        }
    }

    // Перевіряємо, чи завершилось читання даних по I2C
    if (hmpu1.state == MPU_STATE_READY && prev_mpu_state == MPU_STATE_READING_DATA) {

    	// Розраховуємо кут на основі точного часу з TIM2
        MPU6050_Update_Angle(&hmpu1);

        // Синхронізуємо кут модуля з кутом з драйвера
        robot_theta_deg = hmpu1.angle_z;
    }
    prev_mpu_state = hmpu1.state;

    // =================================================================
    // 2. Модуль Одометрії: Розрахунок лінійного шляху через TIM3
    // =================================================================
    uint16_t current_encoder_ticks = __HAL_TIM_GET_COUNTER(&htim3);

    // Дельта імпульсів з урахуванням переповнення 16-бітного таймера (двоєчне доповнення типу int16_t)
    int16_t delta_ticks = (int16_t)(current_encoder_ticks - last_encoder_ticks);
    last_encoder_ticks = current_encoder_ticks;

    // Переводимо кроки енкодера в метри
    float delta_distance_m = (float)delta_ticks * METERS_PER_PULSE;

    // =================================================================
    // 3. Трігонометричне розкладання вектора руху UGV
    // =================================================================
    // Використовуємо поточний кут робота в радіанах
    float theta_rad = robot_theta_deg * (PI_F / 180.0f);

    robot_x += delta_distance_m * cosf(theta_rad);
    robot_y += delta_distance_m * sinf(theta_rad);

    // 4. Оновлюємо дані батареї
    robot_battery_pct = STM32_ReadBatteryPercentage();
}
