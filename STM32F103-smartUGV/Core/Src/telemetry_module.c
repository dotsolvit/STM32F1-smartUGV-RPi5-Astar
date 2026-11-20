/* telemetry_module.c */
#include "telemetry_module.h"
#include "pilot_module.h" // Підключаємо, щоб бачити dir_left та dir_right
#include <math.h>

// Експортуємо структури HAL та MPU з main.c
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

//Структура для збереження відкаліброваних коефіцієнтів та поточних даних HP5883
HP5883_HandleTypeDef compass;

// Створюємо екземпляр драйвера MPU6050
MPU6050_HandleTypeDef hmpu1;

/*
// Оголошення глобальних координат
extern float robot_x;
extern float robot_y;
extern float robot_theta_deg;
extern uint8_t robot_battery_pct;
*/

// Глобальна змінна для Модуля Пілота (щоб він міг бачити відстань у реальному часі)
extern uint16_t sharp_distance_raw_adc;

// Асинхронні лічильники імпульсів з переривань
volatile int32_t left_encoder_total = 0;
volatile int32_t right_encoder_total = 0;

// Змінні для збереження попередніх значень лічильників
static int32_t last_left_ticks = 0;
static int32_t last_right_ticks = 0;
static uint32_t last_mpu_tick = 0;
static MPU_State_t prev_mpu_state = MPU_STATE_UNINIT;

//EMA фільтр для даних від ADC
float Battery_FilterEMA(float new_voltage)
{
    static float filtered_voltage = 0.0f;
    static uint8_t initialized = 0;

    // Коефіцієнт фільтрації Альфа (від 0.0 до 1.0).
    // Чим менше число, тим сильніше згладжування, але більша затримка.
    // Для батареї 0.05 — ідеально (повільна та плавна реакція).
    const float alpha = 0.05f;

    if (!initialized)
    {
        filtered_voltage = new_voltage;
        initialized = 1;
    }
    else
    {
        filtered_voltage +=
            alpha * (new_voltage - filtered_voltage);
    }

    return filtered_voltage;
}

// Допоміжна функція нормалізації кута в межі 0..360°
static float Normalize_Angle_360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f)    angle += 360.0f;
    return angle;
}

// Функція асинхронного послідовного зчитування обох каналів АЦП
void Telemetry_Read_ADC(void) {
    uint16_t adc_ch0 = 0;
    uint16_t adc_ch1 = 0;

    // --- 1. ОПРОС КАНАЛУ 0 (PA0 - БАТАРЕЯ) ---
    ADC_ReadChannel(ADC_CHANNEL_0, &adc_ch0);

    // --- 2. ОПРОС КАНАЛУ 1 (PA1 - SHARP IR) ---
    ADC_ReadChannel(ADC_CHANNEL_1, &adc_ch1);

    // --- 3. ЦИФРОВА ФІЛЬТРАЦІЯ (Експоненційне рухоме середнє) ---
    // Статичні змінні для збереження попереднього відфільтрованого стану
    static float filtered_adc_ch0 = 0;
    filtered_adc_ch0 = Battery_FilterEMA((float)adc_ch0);

    // Розраховуємо вольти на основі ПЛАВНОГО значення filtered_adc_ch0
    float v_pin = (filtered_adc_ch0 * 3.28f) / 4095.0f;
    float v_batt = v_pin * BATT_DIVIDER_RATIO;

    // Розрахунок відсотків заряда батареї
    uint8_t current_pct = 0;
    if (v_batt <= BATT_MIN_VOLTAGE) {
        current_pct = 0;
    } else if (v_batt >= BATT_MAX_VOLTAGE) {
        current_pct = 100;
    } else {
        current_pct = (uint8_t)(((v_batt - BATT_MIN_VOLTAGE) / (BATT_MAX_VOLTAGE - BATT_MIN_VOLTAGE)) * 100.0f);
    }

    // Записуємо у глобальну змінну телеметрії
    robot_battery_pct = current_pct;

    // Зберігаємо значення далекоміра для Пілота
    sharp_distance_raw_adc = adc_ch1;
}

//Функція калібрування компасу
void Telemetry_Calibration(void) {
	//До початку роботи цієї функції вмикаємо обертання машинки
    // Запускаємо калібрування на 6 секунд
    HP5883_RunCalibration(&compass, 6000);
    // Записуємо результати у Flash за допомогою твоїх низькорівневих функцій шаблону:
    HAL_FLASH_Unlock();
    // Обов'язково очищуємо сторінку перед записом!
    // Налаштовуємо структуру для стирання сторінки
    FLASH_EraseInitTypeDef eraseConfig;
    eraseConfig.TypeErase = FLASH_TYPEERASE_PAGES; // Вказуємо, що перемо сторінками
    eraseConfig.PageAddress = FLASH_COMPASS_ADDR;  // Адреса сторінки, яку треба стерти
    eraseConfig.NbPages = 1;                       // Кількість сторінок для стирання (всього 1)

    uint32_t pageError = 0; // Сюди запишеться адреса помилки, якщо стирання провалиться

    //Викликаємо функцію стирання сторінки
    if (HAL_FLASHEx_Erase(&eraseConfig, &pageError) == HAL_OK)
    {
        // Записуємо по 2 байти (Half-Word)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_COMPASS_ADDR, (uint16_t)compass.x_offset);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_COMPASS_ADDR + 2, (uint16_t)compass.y_offset);
    }

    HAL_FLASH_Lock();
    //
    HP5883_Read(&compass);             // Отримуємо свіжий відкалібрований кут (0..360°)
    hmpu1.angle_z = compass.heading;   // Перезаписуємо інтегратор гіроскопа
    robot_theta_deg = compass.heading; // Оновлюємо глобальний курс
}

void Telemetry_Init(void) {
    // 1. Стартуємо канали TIM3 в режимі переривання захоплення (Input Capture Interrupt)
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);

    left_encoder_total = 0;
    right_encoder_total = 0;
    last_left_ticks = 0;
    last_right_ticks = 0;

    // 2. Стартуємо системний мікросекундний хронометр TIM2
    //HAL_TIM_Base_Start(&htim2);
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

    //Іниціалізація HP5883 та зчитування даних калібрування
    //Ініціалізація
    if(HP5883_Init(&compass, &hi2c2)==HAL_OK) {
		//Читаємо збережені раніше коефіцієнти прямо з Flash пам'яті
		compass.x_offset = *(volatile int16_t*)(FLASH_COMPASS_ADDR);
		compass.y_offset = *(volatile int16_t*)(FLASH_COMPASS_ADDR + 2);
		//Валідація: якщо у Flash лежить 0xFFFF (чиста сторінка), значить калібрування ні разу не робилося, обнуляємо
		if(compass.x_offset == (int16_t)0xFFFF && compass.y_offset == (int16_t)0xFFFF)
		{
			compass.x_offset = 0;
			compass.y_offset = 0;
		}
		if(HP5883_Read(&compass)==HAL_OK) {             // Отримуємо свіжий відкалібрований кут (0..360°)
			hmpu1.angle_z = Normalize_Angle_360(compass.heading + 90.0f + WALL_CORNER);   // Перезаписуємо інтегратор гіроскопа+90°+кут стіни
			robot_theta_deg = Normalize_Angle_360(compass.heading + 90.0f + WALL_CORNER); // Оновлюємо глобальний курс+90°+кут стіни
		}
    }

    //Завжди починаємо з координат х=4.0м y=4.0м
    robot_x = 4.0f;
    robot_y = 4.0f;
    robot_battery_pct = 100;
}
//Нова версія з урахуванням напрямку руху
// Колбек HAL, який викликається автоматично при кожному імпульсі на PB4 або PB5
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
            if (dir_left == DIR_BACKWARD) {
                left_encoder_total--;
            } else {
                left_encoder_total++;
            }
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            if (dir_right == DIR_BACKWARD) {
                right_encoder_total--;
            } else {
                right_encoder_total++;
            }
        }
    }
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

        // Опитуємо АЦП паралельно з тиками гіроскопа (теж на частоті 50 Гц)
        Telemetry_Read_ADC();

        if (hmpu1.state == MPU_STATE_READY) {
            hmpu1.state = MPU_STATE_READING_DATA; // Переводимо в стан читання
        }
    }

    // Оновлений блок об'єднання гіроскопа та компаса (Компас опитується на 10 Hz)
	if (hmpu1.state == MPU_STATE_READY && prev_mpu_state == MPU_STATE_READING_DATA) {
		// 1. Оновлюємо швидкий кут за гіроскопом (50 Hz)
		MPU6050_Update_Angle(&hmpu1);
		float gyro_angle = Normalize_Angle_360(hmpu1.angle_z);

		// Лічильник для проріджування запитів до компаса (5 * 20мс = 100мс)
		static uint8_t compass_divider = 0;
		compass_divider++;

		if (compass_divider >= 5) {
			compass_divider = 0; // Скидаємо лічильник

			// 2. Раз на 100 мс читаємо стабільний кут із компаса (як що це не широке сканування, яке
			// впливає на компас)
			if (HP5883_Read(&compass) == HAL_OK && current_robot_status != ROBOT_STATUS_SCANNING) {
				float mag_angle = Normalize_Angle_360(compass.heading + 90.0f + WALL_CORNER); // курс+90°+кут стіни

				// 3. Шукаємо найкоротшу різницю (-180 ... +180)
				float angle_diff = mag_angle - gyro_angle;
				if (angle_diff > 180.0f)  angle_diff -= 360.0f;
				if (angle_diff < -180.0f) angle_diff += 360.0f;

				// 4. Комплементарний фільтр (коефіцієнт 0.08f оптимальний для 10 Hz)
				float fused_angle = Normalize_Angle_360(gyro_angle + 0.08f * angle_diff);

				// 5. Оновлюємо інтегратор гіроскопа та глобальний курс
				hmpu1.angle_z = fused_angle;
				robot_theta_deg = fused_angle;
			} else {
				// Якщо компас не відповів — тримаємо курс за гіроскопом
				hmpu1.angle_z = gyro_angle;
				robot_theta_deg = gyro_angle;
			}
		} else {
			// У проміжках між замірами компаса (4 такти з 5) працюємо суто на швидкому гіроскопі
			hmpu1.angle_z = gyro_angle;
			robot_theta_deg = gyro_angle;
		}
	}
	prev_mpu_state = hmpu1.state;

    // =================================================================
    // 3. Модуль Диференціальної Одометрії
    // =================================================================
    // Захищаємо змінні переривань від зміни під час читання (критична секція)
    __disable_irq();
    int32_t current_left = left_encoder_total;
    int32_t current_right = right_encoder_total;
    __enable_irq();

    // Вираховуємо, скільки імпульсів пройшло з попереднього циклу
    int32_t delta_left = current_left - last_left_ticks;
    int32_t delta_right = current_right - last_right_ticks;

    last_left_ticks = current_left;
    last_right_ticks = current_right;
    ///Нова версія
    // Якщо робот зараз калібрується — не змінюємо глобальні координати X та Y
    if (current_robot_status != ROBOT_STATUS_CALIBRATING) {
        float dist_left_m = (float)delta_left * METERS_PER_PULSE;
        float dist_right_m = (float)delta_right * METERS_PER_PULSE;
        float delta_center_distance_m = (dist_left_m + dist_right_m) / 2.0f;

        float theta_rad = robot_theta_deg * (PI_F / 180.0f);

        // Навігаційна система координат (0° = Північ/+Y, 90° = Схід/+X)
        robot_x += delta_center_distance_m * sinf(theta_rad);
        robot_y += delta_center_distance_m * cosf(theta_rad);
    }

}
