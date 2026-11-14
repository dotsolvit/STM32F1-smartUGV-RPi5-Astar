/* pilot_module.c */
#include "pilot_module.h"
#include "telemetry_module.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern volatile RobotStatus_t current_robot_status;

// Глобальна змінна для Модуля Пілота (щоб він міг бачити відстань у реальному часі)
extern uint16_t sharp_distance_raw_adc;

// Цільові швидкості, які хоче отримати навігатор (-99 ... 99)
static int16_t target_speed_left = 0;
static int16_t target_speed_right = 0;

// Поточні швидкості, які реально зараз подаються на ШІМ (0 ... 99)
static int16_t current_speed_left = 0;
static int16_t current_speed_right = 0;

// Поточні напрямки руху бортів
volatile MotorDirection_t dir_left = DIR_STOP;
volatile MotorDirection_t dir_right = DIR_STOP;

// Прототип функції відправки точки сканування з main.c
extern void STM32_SendScanPoint(int16_t angle, uint16_t distance_mm, uint8_t is_last);

//
// Експорт масиву маршруту з main.c / stm32f1xx_it.c
extern volatile PathPoint_t robot_path[];
extern volatile uint8_t robot_path_size;
extern volatile uint8_t robot_new_path_ready;
extern volatile RobotState_t current_robot_state;
extern volatile RobotStatus_t current_robot_status;

static NavState_t nav_state = NAV_IDLE;
static uint16_t current_wp_idx = 0;


// Змінні сканера
static int16_t scan_angle = 0;
static int8_t narrow_dir = 1;
static uint32_t servo_timer = 0;
static uint8_t servo_first_move = 0;

// Таблиця калібрування Sharp GP2Y0A02YK0F (усереднені значення АЦП з таблиці калібрування)
static const uint16_t sharp_adc_table[14] = {
    3109, // 200 мм
    2517, // 300 мм
    1894, // 400 мм
    1504, // 500 мм
    1232, // 600 мм
    1042, // 700 мм
    929,  // 800 мм
    794,  // 900 мм
    697,  // 1000 мм
    627,  // 1100 мм
    570,  // 1200 мм
    503,  // 1300 мм
    488,  // 1400 мм
    430   // 1500 мм
};

// ВСТАНОВЛЕННЯ КУТА СЕРВОПРИВОДУ (-90 ... +90)
void Pilot_SetServoAngle(int16_t angle_deg) {
    if (angle_deg < -90) angle_deg = -90;
    if (angle_deg > 90)  angle_deg = 90;

    // Лінійний перерахунок кута (-90..+90) у ширину імпульсу (450..2390 мкс)
    uint32_t pulse = SERVO_MAX_PULSE - ((uint32_t)(angle_deg + 90) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) / 180;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

// КОНВЕРТАЦІЯ СИРОГО АЦП SHARP IR У МІЛІМЕТРИ
uint16_t Pilot_GetSharpDistanceMM(void) {
    uint16_t raw = sharp_distance_raw_adc;

    // Граничні значення
    if (raw >= sharp_adc_table[0]) return 200;  // Ближче 200 мм (або початок зворотної гілки кривої)
    if (raw <= sharp_adc_table[13]) return 1500; // Далі 1500 мм (перешкод немає)

    // Лінійна інтерполяція між сусідніми точками таблиці (крок 100 мм)
    for (uint8_t i = 0; i < 13; i++) {
        if (raw <= sharp_adc_table[i] && raw > sharp_adc_table[i + 1]) {
            uint16_t dist_base = (i + 2) * 100; // 200, 300, ... 1400 мм
            uint16_t adc_span = sharp_adc_table[i] - sharp_adc_table[i + 1];
            uint16_t adc_diff = sharp_adc_table[i] - raw;

            return dist_base + (uint16_t)(((uint32_t)adc_diff * 100) / adc_span);
        }
    }
    return 1500;
}
// Допоміжна функція: найкоротша різниця кутів (-180 ... +180)
static float Get_Angle_Error(float target_deg, float current_deg) {
    float err = target_deg - current_deg;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// Основний диспетчер навігації та сканування (викликається всередині Pilot_Update)
static void Pilot_Navigation_Process(void) {
    uint32_t now = HAL_GetTick();

    // 1. Обробка зовнішньої команди СТОП або КАЛІБРУВАННЯ
    if (current_robot_state != ROBOT_STATE_MOVING) {
        if (nav_state != NAV_IDLE && nav_state != NAV_WIDE_SCANNING) {
            nav_state = NAV_IDLE;
            Pilot_SetTargetSpeed(0, 0);
            Pilot_SetServoAngle(0);
        }
        return;
    }

    // 2. Ініціалізація нового маршруту при отриманні команди START
    if (nav_state == NAV_IDLE) {
        if (robot_path_size > 0) {
            // Якщо перша точка маршруту A* — це поточна позиція робота, переходимо одразу до другої
            current_wp_idx = (robot_path_size > 1) ? 1 : 0;
            nav_state = NAV_TURNING;
            current_robot_status = ROBOT_STATUS_MOVING;
            Pilot_SetServoAngle(0);
        } else {
            // Маршруту немає — переводимо стейт у стоп
            current_robot_state = ROBOT_STATE_STOPPED;
            return;
        }
    }

    // Отримуємо координати поточної цільової точки повороту
    float target_x = robot_path[current_wp_idx].x;
    float target_y = robot_path[current_wp_idx].y;

    float dx = target_x - robot_x;
    float dy = target_y - robot_y;
    float dist_to_wp_m = sqrtf(dx * dx + dy * dy);

    // Навігаційний азимут на ціль (0° = Північ/+Y, 90° = Схід/+X)
    float target_heading = atan2f(dx, dy) * (180.0f / PI_F);
    if (target_heading < 0.0f) target_heading += 360.0f;

    float heading_err = Get_Angle_Error(target_heading, robot_theta_deg);

    // =================================================================
    // ФАЗА 1: ПОВОРОТ НА МІСЦІ НА НАСТУПНУ ТОЧКУ
    // =================================================================
    if (nav_state == NAV_TURNING) {
        Pilot_SetServoAngle(0); // Під час повороту голова дивиться строго вперед

        if (fabsf(heading_err) <= HEADING_TOLERANCE_DEG) {
            // Розворот завершено! Переходимо до руху по прямій
            Pilot_SetTargetSpeed(0, 0);
            nav_state = NAV_DRIVING;

            // Готуємо вузьке сканування (-11° ... +11°)
            scan_angle = 0;
            narrow_dir = 1;
            servo_timer = now;
        } else {
            // Пропорційне уповільнення при наближенні до цільового кута
            int16_t turn_pwm = (int16_t)(fabsf(heading_err) * 1.5f);
            if (turn_pwm > TURN_MAX_SPEED) turn_pwm = TURN_MAX_SPEED;
            if (turn_pwm < MOTOR_MIN_PWM)  turn_pwm = MOTOR_MIN_PWM;

            if (heading_err > 0) {
                // Поворот за годинниковою стрілкою (праворуч)
                Pilot_SetTargetSpeed(turn_pwm, -turn_pwm);
            } else {
                // Поворот проти годинникової стрілки (ліворуч)
                Pilot_SetTargetSpeed(-turn_pwm, turn_pwm);
            }
        }
    }

    // =================================================================
    // ФАЗА 2: РУХ ПО ПРЯМОМУ ВІДРІЗКУ + ВУЗЬКЕ СКАНУВАННЯ
    // =================================================================
    else if (nav_state == NAV_DRIVING) {
        // А. Перевірка досягнення точки маршруту
        if (dist_to_wp_m <= WAYPOINT_TOLERANCE_M) {
            current_wp_idx++;
            if (current_wp_idx >= robot_path_size) {
                // Фінальна точка маршруту досягнута!
                Pilot_SetTargetSpeed(0, 0);
                Pilot_SetServoAngle(0);
                nav_state = NAV_IDLE;
                current_robot_state = ROBOT_STATE_STOPPED;
                current_robot_status = ROBOT_STATUS_ARRIVED;
                return;
            } else {
                // Переходимо до повороту на наступний відрізок
                Pilot_SetTargetSpeed(0, 0);
                nav_state = NAV_TURNING;
                return;
            }
        }

        // Б. Вузьке сканування попереду (-11° ... 0° ... +11°)
        if (now - servo_timer >= 90) { // 90 мс достатньо для кроку в 11 градусів
            uint16_t measured_mm = Pilot_GetSharpDistanceMM();

            // Дистанція до точки повороту в мм з урахуванням вильоту сервоприводу на 60 мм
            float dist_to_wp_from_servo_mm = (dist_to_wp_m * 1000.0f) - SERVO_OFFSET_MM;

            // Перевіряємо перешкоду ТІЛЬКИ якщо вона ближче кінця поточного відрізка
            if (dist_to_wp_from_servo_mm > 0.0f && (float)measured_mm < dist_to_wp_from_servo_mm) {
                // Якщо перешкода в небезпечній зоні (або ближче ніж 250 мм)
                if (measured_mm <= 300) {
                    Pilot_SetTargetSpeed(0, 0); // Команда на плавну зупинку
                    nav_state = NAV_OBSTACLE_STOPPING;
                    return;
                }
            }

            // Похитування головою: -11 -> 0 -> +11 -> 0 -> -11
            scan_angle += (NARROW_SCAN_MAX_DEG * narrow_dir);
            if (scan_angle >= NARROW_SCAN_MAX_DEG) {
                scan_angle = NARROW_SCAN_MAX_DEG;
                narrow_dir = -1;
            } else if (scan_angle <= -NARROW_SCAN_MAX_DEG) {
                scan_angle = -NARROW_SCAN_MAX_DEG;
                narrow_dir = 1;
            }
            Pilot_SetServoAngle(scan_angle);
            servo_timer = now;
        }

        // В. П-регулятор утримання курсу на прямій
        // Якщо робот відхилився від лінії маршруту, злегка підгальмовуємо відповідний борт
        float kp = 1.8f;
        int16_t correction = (int16_t)(heading_err * kp);

        int16_t left_pwm  = DRIVE_BASE_SPEED + correction;
        int16_t right_pwm = DRIVE_BASE_SPEED - correction;

        // Обмежуємо ШІМ в робочих межах
        if (left_pwm > MOTOR_MAX_PWM)  left_pwm = MOTOR_MAX_PWM;
        if (left_pwm < MOTOR_MIN_PWM)  left_pwm = MOTOR_MIN_PWM;
        if (right_pwm > MOTOR_MAX_PWM) right_pwm = MOTOR_MAX_PWM;
        if (right_pwm < MOTOR_MIN_PWM) right_pwm = MOTOR_MIN_PWM;

        Pilot_SetTargetSpeed(left_pwm, right_pwm);
    }

    // =================================================================
    // ФАЗА 3: ОЧІКУВАННЯ ПОВНОЇ ЗУПИНКИ ПЕРЕД ШИРОКИМ СКАНУВАННЯМ
    // =================================================================
    else if (nav_state == NAV_OBSTACLE_STOPPING) {
        // Чекаємо, поки ШІМ впаде до 0 і колеса повністю зупиняться по інерції
        if (current_speed_left == 0 && current_speed_right == 0 && dir_left == DIR_STOP && dir_right == DIR_STOP) {
            nav_state = NAV_WIDE_SCANNING;
            current_robot_status = ROBOT_STATUS_SCANNING;

            // Ставимо голову в крайнє положення -90° для початку панорами
            scan_angle = -90;
            Pilot_SetServoAngle(scan_angle);
            servo_timer = now;
            servo_first_move = 1;
        }
    }

    // =================================================================
    // ФАЗА 4: ШИРОКЕ ПАНОРАМНЕ СКАНУВАННЯ (-90° ... +90°)
    // =================================================================
    else if (nav_state == NAV_WIDE_SCANNING) {
        // Перший поворот з центру в -90° потребує більше часу (350 мс), крок по 6° — 65 мс
        uint32_t wait_ms = servo_first_move ? 350 : 65;

        if (now - servo_timer >= wait_ms) {
            servo_first_move = 0;
            uint16_t dist_mm = Pilot_GetSharpDistanceMM();
            uint8_t is_last = (scan_angle >= 90) ? 1 : 0;

            // Відправляємо чисті виміряні міліметри без віднімання 60 мм (сервер врахує сам)
            STM32_SendScanPoint(scan_angle, dist_mm, is_last);

            if (is_last) {
                // Панораму завершено! Повертаємо голову вперед і чекаємо новий маршрут A* від RPi5
                Pilot_SetServoAngle(0);
                nav_state = NAV_IDLE;
                current_robot_state = ROBOT_STATE_STOPPED;
                current_robot_status = ROBOT_STATUS_STOPPED;
            } else {
                scan_angle += WIDE_SCAN_STEP_DEG;
                Pilot_SetServoAngle(scan_angle);
                servo_timer = now;
            }
        }
    }
}


//====================

void Pilot_Init(void) {
    // Стартуємо апаратний ШІМ на всіх 4-х каналах TIM4
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // Примусовий стоп на старті
    Pilot_SetTargetSpeed(0, 0);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Запуск ШІМ сервоприводу на PA8
    Pilot_SetServoAngle(0);                   // Ставимо голову рівно по центру

}

// Внутрішня функція апаратного встановлення ШІМ у регістри порівняння таймера (CCR)
static void Pilot_HardwareWrite(int16_t left_pwm, MotorDirection_t left_dir,
                                 int16_t right_pwm, MotorDirection_t right_dir) {
    // --- КЕРУВАННЯ ЛІВИМ БОРТОМ (CH1 = PB6, CH2 = PB7) ---
    if (left_dir == DIR_FORWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    } else if (left_dir == DIR_BACKWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, left_pwm);
    } else { // Стоп
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }

    // --- КЕРУВАННЯ ПРАВИМ БОРТОМ (CH3 = PB8, CH4 = PB9) ---
    if (right_dir == DIR_FORWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    } else if (right_dir == DIR_BACKWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, right_pwm);
    } else { // Стоп
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    }
}

// Відкритий інтерфейс для зміни цільової швидкості (-99...99)
void Pilot_SetTargetSpeed(int16_t left, int16_t right) {
    if (left > MOTOR_MAX_PWM)  left = MOTOR_MAX_PWM;
    if (left < -MOTOR_MAX_PWM) left = -MOTOR_MAX_PWM;
    if (right > MOTOR_MAX_PWM)  right = MOTOR_MAX_PWM;
    if (right < -MOTOR_MAX_PWM) right = -MOTOR_MAX_PWM;

    target_speed_left = left;
    target_speed_right = right;
}

// Основний періодичний забіг модуля (викликається в main.c кожні 20 мс)
void Pilot_Update(void) {
    static uint32_t last_pilot_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    Pilot_Navigation_Process();                  // Основний диспетчер навігації та сканування

    // Виконуємо розрахунок профілю швидкості строго раз на 20 мс (50 Гц)
    if (current_tick - last_pilot_tick < 20) return;
    last_pilot_tick = current_tick;

    // --- 1. ПЛАВНЕ РЕГУЛЮВАННЯ ЛІВОГО БОРТУ ---
    int16_t req_left_pwm = (target_speed_left >= 0) ? target_speed_left : -target_speed_left;
    MotorDirection_t req_left_dir = (target_speed_left > 0) ? DIR_FORWARD : ((target_speed_left < 0) ? DIR_BACKWARD : DIR_STOP);

    if (dir_left != req_left_dir && dir_left != DIR_STOP) {
        // Якщо потрібно різко змінити напрямок, спочатку плавно гальмуємо до нуля
        current_speed_left -= MOTOR_DECEL_STEP;
        if (current_speed_left <= 0) {
            current_speed_left = 0;
            dir_left = DIR_STOP;
        }
    } else {
        dir_left = req_left_dir;
        if (current_speed_left < req_left_pwm) {
            current_speed_left += MOTOR_ACCEL_STEP;
            if (current_speed_left > req_left_pwm) current_speed_left = req_left_pwm;
        } else if (current_speed_left > req_left_pwm) {
            current_speed_left -= MOTOR_DECEL_STEP;
            if (current_speed_left < req_left_pwm) current_speed_left = req_left_pwm;
        }
    }

    // --- 2. ПЛАВНЕ РЕГУЛЮВАННЯ ПРАВОГО БОРТУ ---
    int16_t req_right_pwm = (target_speed_right >= 0) ? target_speed_right : -target_speed_right;
    MotorDirection_t req_right_dir = (target_speed_right > 0) ? DIR_FORWARD : ((target_speed_right < 0) ? DIR_BACKWARD : DIR_STOP);

    if (dir_right != req_right_dir && dir_right != DIR_STOP) {
        current_speed_right -= MOTOR_DECEL_STEP;
        if (current_speed_right <= 0) {
            current_speed_right = 0;
            dir_right = DIR_STOP;
        }
    } else {
        dir_right = req_right_dir;
        if (current_speed_right < req_right_pwm) {
            current_speed_right += MOTOR_ACCEL_STEP;
            if (current_speed_right > req_right_pwm) current_speed_right = req_right_pwm;
        } else if (current_speed_right > req_right_pwm) {
            current_speed_right -= MOTOR_DECEL_STEP;
            if (current_speed_right < req_right_pwm) current_speed_right = req_right_pwm;
        }
    }

    // --- 3. ВРАХУВАННЯ ЕФЕКТУ ІНЕРЦІЇ ДЛЯ СТАТУСУ ---
    // Якщо двигуни вимкнено (target=0), але колеса по інерції крутяться (одометрія фіксує імпульси),
    // або ШІМ ще плавно падає, ми НЕ вважаємо, що робот повністю зупинився.
    if (current_speed_left == 0 && current_speed_right == 0) {
        // Додатково перевіряємо, чи заспокоїлась одометрія з модуля телеметрії
        extern volatile int32_t left_encoder_total;
        static int32_t check_last_ticks = 0;

        if (left_encoder_total == check_last_ticks) {
            // Реальна повна зупинка
            if (current_robot_status == ROBOT_STATUS_MOVING) {
                current_robot_status = ROBOT_STATUS_STOPPED;
            }
        }
        check_last_ticks = left_encoder_total;
    }

    // Зрізаємо "мертву зону" ШІМ (якщо швидкість замала для зрушення, примусово ставимо 0, щоб двигуни не пищали)
    int16_t out_left = (current_speed_left < MOTOR_MIN_PWM && current_speed_left > 0) ? 0 : current_speed_left;
    int16_t out_right = (current_speed_right < MOTOR_MIN_PWM && current_speed_right > 0) ? 0 : current_speed_right;

    // --- 4. ЗАПИС В РЕГІСТРИ ТАЙМЕРА ---
    Pilot_HardwareWrite(out_left, dir_left, out_right, dir_right);
}
