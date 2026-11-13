/* pilot_module.c */
#include "pilot_module.h"
#include "telemetry_module.h"

extern TIM_HandleTypeDef htim4;

// Цільові швидкості, які хоче отримати навігатор (-99 ... 99)
static int16_t target_speed_left = 0;
static int16_t target_speed_right = 0;

// Поточні швидкості, які реально зараз подаються на ШІМ (0 ... 99)
static int16_t current_speed_left = 0;
static int16_t current_speed_right = 0;

// Поточні напрямки руху бортів
volatile MotorDirection_t dir_left = DIR_STOP;
volatile MotorDirection_t dir_right = DIR_STOP;

void Pilot_Init(void) {
    // Стартуємо апаратний ШІМ на всіх 4-х каналах TIM4
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    // Примусовий стоп на старті
    Pilot_SetTargetSpeed(0, 0);
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
