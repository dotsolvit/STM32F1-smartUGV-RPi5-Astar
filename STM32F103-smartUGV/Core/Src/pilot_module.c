/* pilot_module.c */
#include "pilot_module.h"
#include "telemetry_module.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

// Глобальна змінна для Модуля Пілота (щоб він міг бачити відстань у реальному часі)
extern uint16_t sharp_distance_raw_adc;

// Експорт змінних стану та маршруту з main.c / stm32f1xx_it.c
extern volatile PathPoint_t robot_path[];
extern volatile uint8_t robot_path_size;
extern volatile uint8_t robot_new_path_ready;
extern volatile RobotState_t current_robot_state;
extern volatile RobotStatus_t current_robot_status;

// Експорт лічильників енкодерів з telemetry_module.c
extern volatile int32_t left_encoder_total;
extern volatile int32_t right_encoder_total;

// Прототип функції відправки точки сканування з main.c
extern void STM32_SendScanPoint(int16_t angle, uint16_t distance_mm, uint8_t is_last);

// Змінні керування двигунами
// Цільові швидкості, які хоче отримати навігатор (-99 ... 99)
static int16_t target_speed_left = 0;
static int16_t target_speed_right = 0;
// Поточні швидкості, які реально зараз подаються на ШІМ (0 ... 99)
static int16_t current_speed_left = 0;
static int16_t current_speed_right = 0;

// Поточні напрямки руху бортів
volatile MotorDirection_t dir_left = DIR_STOP;
volatile MotorDirection_t dir_right = DIR_STOP;

// Змінні навігатора
static NavState_t nav_state = NAV_IDLE;
static StopReason_t stop_reason = STOP_REASON_NONE;
static uint16_t current_wp_idx = 0;
static uint8_t has_valid_path = 0;

// Змінні контролю інерції
static int32_t coast_last_left = 0;
static int32_t coast_last_right = 0;
static uint32_t coast_still_timer = 0;
static uint32_t coast_start_timer = 0;

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
    if (raw > sharp_adc_table[0]) return 0;  // Ближче 200 мм (або початок зворотної гілки кривої)
    if (raw < sharp_adc_table[13]) return 2000; // Далі 1500 мм (перешкод немає)

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

// --- АПАРАТНИЙ ЗАПИС ШІМ У РЕГІСТРИ TIM4 ---
static void Pilot_HardwareWrite(int16_t left_pwm, MotorDirection_t l_dir,
                                int16_t right_pwm, MotorDirection_t r_dir) {
    // Лівий борт (CH1 = PB6, CH2 = PB7)
    if (l_dir == DIR_FORWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    } else if (l_dir == DIR_BACKWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, left_pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }

    // Правий борт (CH3 = PB8, CH4 = PB9) — з урахуванням зворотної розпайки
    if (r_dir == DIR_FORWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, right_pwm);
    } else if (r_dir == DIR_BACKWARD) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    }
}

//Нова версія (з інерцією) == v3
//Ініціалізація ШІМ, швидкість 0, серво - прямо
void Pilot_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    Pilot_StopImmediate();
    Pilot_SetServoAngle(0);
}

// Відкритий інтерфейс для зміни цільової швидкості (-99...99)
void Pilot_SetTargetSpeed(int16_t left, int16_t right) {
    if (left > MOTOR_MAX_PWM)   left = MOTOR_MAX_PWM;
    if (left < -MOTOR_MAX_PWM)  left = -MOTOR_MAX_PWM;
    if (right > MOTOR_MAX_PWM)  right = MOTOR_MAX_PWM;
    if (right < -MOTOR_MAX_PWM) right = -MOTOR_MAX_PWM;

    target_speed_left = left;
    target_speed_right = right;
}

// Миттєве зняття напруги з двигунів (для перешкод та екстрених зупинок)
void Pilot_StopImmediate(void) {
    target_speed_left = 0;
    target_speed_right = 0;
    current_speed_left = 0;
    current_speed_right = 0;
    dir_left = DIR_STOP;
    dir_right = DIR_STOP;
    Pilot_HardwareWrite(0, DIR_STOP, 0, DIR_STOP);
}

// Допоміжна функція переходу у фазу інерційного вибігу
static void Enter_Coasting_State(StopReason_t reason, uint8_t immediate_cut) {
    stop_reason = reason;
    nav_state = NAV_COASTING;

    if (immediate_cut) {
        Pilot_StopImmediate();
    } else {
        Pilot_SetTargetSpeed(0, 0);
    }

    __disable_irq();
    coast_last_left = left_encoder_total;
    coast_last_right = right_encoder_total;
    __enable_irq();

    uint32_t now = HAL_GetTick();
    coast_still_timer = now;
    coast_start_timer = now;
}

// --- АВТОМАТ НАВІГАЦІЇ ТА СКАНУВАННЯ ---
static void Pilot_Navigation_Process(void) {
    uint32_t now = HAL_GetTick();

    // 1. Обробка нового маршруту A* на ходу
    if (robot_new_path_ready == 1) {
        robot_new_path_ready = 0;
        if (robot_path_size > 0) {
            has_valid_path = 1;
            current_wp_idx = (robot_path_size > 1) ? 1 : 0;
            if (current_robot_state == ROBOT_STATE_MOVING) {
                nav_state = NAV_TURNING;
                current_robot_status = ROBOT_STATUS_MOVING;
                Pilot_SetServoAngle(0);
            }
        } else {
            has_valid_path = 0;
        }
    }

    // 2. Обробка зовнішньої команди СТОП або КАЛІБРУВАННЯ
    if (current_robot_state != ROBOT_STATE_MOVING) {
        if (nav_state == NAV_TURNING || nav_state == NAV_DRIVING) {
            Pilot_SetServoAngle(0);
            Enter_Coasting_State(STOP_REASON_USER_CMD, 0);
            return;
        }
        if (nav_state != NAV_COASTING && nav_state != NAV_WIDE_SCANNING) {
            nav_state = NAV_IDLE;
            return;
        }
    }

    // 3. Очікування старту
    if (nav_state == NAV_IDLE) {
        if (current_robot_state == ROBOT_STATE_MOVING) {
            if (has_valid_path && robot_path_size > 0) {
                nav_state = NAV_TURNING;
                current_robot_status = ROBOT_STATUS_MOVING;
                Pilot_SetServoAngle(0);
            } else {
                current_robot_state = ROBOT_STATE_STOPPED;
                current_robot_status = ROBOT_STATUS_STOPPED;
            }
        }
        return;
    }

    // 4. Фаза інерційного вибігу та фіксації повної зупинки
    if (nav_state == NAV_COASTING) {
        if (current_speed_left == 0 && current_speed_right == 0) {
            __disable_irq();
            int32_t cur_l = left_encoder_total;
            int32_t cur_r = right_encoder_total;
            __enable_irq();

            if (cur_l != coast_last_left || cur_r != coast_last_right) {
                coast_last_left = cur_l;
                coast_last_right = cur_r;
                coast_still_timer = now;
            }

            if ((now - coast_still_timer >= STOP_CONFIRM_TIME_MS) || (now - coast_start_timer >= STOP_TIMEOUT_MS)) {
                if (stop_reason == STOP_REASON_WAYPOINT) {
                    nav_state = NAV_TURNING;
                }
                else if (stop_reason == STOP_REASON_ARRIVED) {
                    nav_state = NAV_IDLE;
                    has_valid_path = 0;
                    current_robot_state = ROBOT_STATE_STOPPED;
                    current_robot_status = ROBOT_STATUS_ARRIVED;
                }
                else if (stop_reason == STOP_REASON_OBSTACLE) {
                    nav_state = NAV_WIDE_SCANNING;
                    current_robot_status = ROBOT_STATUS_SCANNING;
                    scan_angle = -90;
                    Pilot_SetServoAngle(scan_angle);
                    servo_timer = now;
                    servo_first_move = 1;
                }
                else {
                    nav_state = NAV_IDLE;
                    current_robot_status = ROBOT_STATUS_STOPPED;
                }
            }
        } else {
            coast_still_timer = now;
        }
        return;
    }

    // 5. Фаза широкого панорамного сканування (-90° ... +90°)
    if (nav_state == NAV_WIDE_SCANNING) {
        uint32_t wait_ms = servo_first_move ? 350 : NARROW_SCAN_DELAY_MS; //Тіж 135 мс

        if (now - servo_timer >= wait_ms) {
            servo_first_move = 0;
            uint16_t dist_mm = Pilot_GetSharpDistanceMM();
            uint8_t is_last = (scan_angle >= 90) ? 1 : 0;

            STM32_SendScanPoint(scan_angle, dist_mm, is_last);

            if (is_last) {
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
        return;
    }

    // Розрахунок геометрії до поточної цільової точки
    float target_x = robot_path[current_wp_idx].x;
    float target_y = robot_path[current_wp_idx].y;

    float dx = target_x - robot_x;
    float dy = target_y - robot_y;
    float dist_to_wp_m = sqrtf(dx * dx + dy * dy);

    float target_heading = atan2f(dx, dy) * (180.0f / PI_F);
    if (target_heading < 0.0f) target_heading += 360.0f;
    float heading_err = Get_Angle_Error(target_heading, robot_theta_deg);

    // 6. Фаза повороту на місці
    if (nav_state == NAV_TURNING) {
        Pilot_SetServoAngle(0);

        if (fabsf(heading_err) <= HEADING_TOLERANCE_DEG) {
            Pilot_StopImmediate();
            nav_state = NAV_DRIVING;
            scan_angle = 0;
            narrow_dir = 1;
            servo_timer = now;
        } else {
            // Пропорційне уповільнення при наближенні до цільового кута
            int16_t turn_pwm = (int16_t)(fabsf(heading_err) * 4.0f);  //Обрали між 1.5f та 6.5f (68/6.5=10,5°)
            if (turn_pwm > TURN_MAX_SPEED) turn_pwm = TURN_MAX_SPEED;
            if (turn_pwm < MOTOR_MIN_PWM)  turn_pwm = MOTOR_MIN_PWM;

            if (heading_err > 0) {
                Pilot_SetTargetSpeed(turn_pwm, -turn_pwm);
            } else {
                Pilot_SetTargetSpeed(-turn_pwm, turn_pwm);
            }
        }
    }

    // 7. Фаза руху по прямому відрізку + вузьке сканування + зона уповільнення
    else if (nav_state == NAV_DRIVING) {
        // А. Перевірка досягнення точки маршруту
        if (dist_to_wp_m <= WAYPOINT_TOLERANCE_M) {
            Pilot_SetServoAngle(0);
            current_wp_idx++;
            if (current_wp_idx >= robot_path_size) {
                Enter_Coasting_State(STOP_REASON_ARRIVED, 1);
            } else {
                Enter_Coasting_State(STOP_REASON_WAYPOINT, 1);
            }
            return;
        }

        // Б. Вузьке сканування (затримка 135 мс)
        if (now - servo_timer >= NARROW_SCAN_DELAY_MS) {
            uint16_t measured_mm = Pilot_GetSharpDistanceMM();
            //Контроль реальності вимірювання
            if(measured_mm >= 200 && measured_mm <= 1500) {
				float dist_to_wp_from_servo_mm = (dist_to_wp_m * 1000.0f) - SERVO_OFFSET_MM;

				if (dist_to_wp_from_servo_mm > 0.0f && (float)measured_mm < dist_to_wp_from_servo_mm) {
					if (measured_mm <= OBSTACLE_STOP_DIST_MM) {
						has_valid_path = 0;
						Enter_Coasting_State(STOP_REASON_OBSTACLE, 1);
						return;
					}
				}
            }
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

        // В. Розрахунок базової швидкості з урахуванням зони уповільнення (0.15 м)
        int16_t base_speed = DRIVE_BASE_SPEED;
        if (dist_to_wp_m < SLOWDOWN_DISTANCE_M) {
            float ratio = dist_to_wp_m / SLOWDOWN_DISTANCE_M;
            base_speed = MOTOR_MIN_PWM; // + (int16_t)((DRIVE_BASE_SPEED - MOTOR_MIN_PWM) * ratio); Прибрав це
        }

        // Г. П-регулятор утримання курсу
        float kp = 1.8f;                           //Подумати над коєфіцієнтом ????
        int16_t correction = (int16_t)(heading_err * kp);

        int16_t left_pwm  = base_speed + correction;
        int16_t right_pwm = base_speed - correction;

        if (left_pwm > MOTOR_MAX_PWM)  left_pwm = MOTOR_MAX_PWM;
        if (left_pwm < MOTOR_MIN_PWM)  left_pwm = MOTOR_MIN_PWM;
        if (right_pwm > MOTOR_MAX_PWM) right_pwm = MOTOR_MAX_PWM;
        if (right_pwm < MOTOR_MIN_PWM) right_pwm = MOTOR_MIN_PWM;

        Pilot_SetTargetSpeed(left_pwm, right_pwm);
    }
}

// --- ДОПОМІЖНА ФУНКЦІЯ ПРОФІЛЮВАННЯ ШВИДКОСТІ ОДНОГО БОРТУ ---
static void Update_Motor_Channel(int16_t target_speed, int16_t *cur_speed, volatile MotorDirection_t *cur_dir) {
    int16_t req_pwm = (target_speed >= 0) ? target_speed : -target_speed;
    MotorDirection_t req_dir = (target_speed > 0) ? DIR_FORWARD : ((target_speed < 0) ? DIR_BACKWARD : DIR_STOP);

    if (*cur_dir != req_dir && *cur_dir != DIR_STOP) {
        *cur_speed -= MOTOR_DECEL_STEP;
        if (*cur_speed < MOTOR_MIN_PWM) {
            *cur_speed = 0;
            *cur_dir = DIR_STOP;
        }
        return;
    }

    if (req_dir == DIR_STOP) {
        if (*cur_speed > 0) {
            *cur_speed -= MOTOR_DECEL_STEP;
            if (*cur_speed < MOTOR_MIN_PWM) {
                *cur_speed = 0;
                *cur_dir = DIR_STOP;
            }
        }
    } else {
        *cur_dir = req_dir;
        if (*cur_speed == 0 && req_pwm > 0) {
            *cur_speed = MOTOR_MIN_PWM;
        }

        if (*cur_speed < req_pwm) {
            *cur_speed += MOTOR_ACCEL_STEP;
            if (*cur_speed > req_pwm) *cur_speed = req_pwm;
        } else if (*cur_speed > req_pwm) {
            *cur_speed -= MOTOR_DECEL_STEP;
            if (*cur_speed < req_pwm) *cur_speed = req_pwm;
        }
    }
}

// --- ГОЛОВНИЙ ЦИКЛ МОДУЛЯ ПІЛОТА (50 Гц) ---
void Pilot_Update(void) {
    static uint32_t last_pilot_tick = 0;
    uint32_t current_tick = HAL_GetTick();

    if (current_tick - last_pilot_tick < 20) return;
    last_pilot_tick = current_tick;

    Pilot_Navigation_Process();

    Update_Motor_Channel(target_speed_left, &current_speed_left, &dir_left);
    Update_Motor_Channel(target_speed_right, &current_speed_right, &dir_right);

    Pilot_HardwareWrite(current_speed_left, dir_left, current_speed_right, dir_right);
}

//===================================

