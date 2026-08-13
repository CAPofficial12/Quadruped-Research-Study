// ─────────────────────────────────────────────
//  main.cpp  –  STM32F405 top-level integration
//
//  TIMER ARCHITECTURE:
//  ┌──────────────────────────────────────────┐
//  │ TIM6 ISR @ 1kHz  (BALANCE_LOOP_HZ)      │
//  │  • Read IMU (if DRDY set)                │
//  │  • Run attitude filter                   │
//  │  • Run balance PID/LQR                   │
//  │  • Update motor outputs                  │
//  │  • Update encoders                       │
//  └──────────────────────────────────────────┘
//  ┌──────────────────────────────────────────┐
//  │ TIM7 ISR @ 50Hz  (GAIT_LOOP_HZ)         │
//  │  • Update gait FSM                       │
//  │  • Update servo interpolators            │
//  │  • Process comms with ESP32              │
//  └──────────────────────────────────────────┘
//  ┌──────────────────────────────────────────┐
//  │ MAIN LOOP (background, non-real-time)    │
//  │  • Serial tuning interface               │
//  │  • Watchdog kick                         │
//  │  • Debug telemetry                       │
//  └──────────────────────────────────────────┘
//
//  IMPORTANT: The balance ISR must NEVER be
//  interrupted by the gait ISR or main loop.
//  Set TIM6 interrupt priority higher than TIM7.
//  In NVIC: TIM6 priority = 0, TIM7 priority = 1
// ─────────────────────────────────────────────

#include "stm32f4xx_hal.h"
#include "config.h"
#include "imu.h"
#include "filter.h"
#include "servo.h"
#include "motor.h"
#include "balance.h"
#include "gait.h"
#include "comms.h"
#include "tuning.h"

// ── HAL handles (from CubeMX-generated code) ──
SPI_HandleTypeDef  hspi1;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart1;   // tuning (VCP)
UART_HandleTypeDef huart2;   // ESP32 comms
TIM_HandleTypeDef  htim6;    // balance loop timer
TIM_HandleTypeDef  htim7;    // gait loop timer

// ── Shared state between ISR and main ─────────
// Mark volatile — modified in ISR, read in main
static volatile Attitude   attitude;
static volatile IMUData    imu_data;
static volatile GaitOutput gait_output;

// Controller commands (received from ESP32)
static volatile float cmd_velocity    = 0.0f;   // m/s
static volatile float cmd_yaw_rate    = 0.0f;   // rad/s
static volatile bool  cmd_walk_mode   = false;

// Watchdog tracking
static volatile uint32_t last_comms_ms = 0;
static volatile bool     comms_alive   = false;

// ── TIM6 ISR — 1kHz balance loop ─────────────
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {

    if (htim->Instance == TIM6) {
        // ── BALANCE LOOP (1kHz) ────────────────
        // This must complete in < 1ms.
        // On STM32F405 @ 168MHz that's 168,000 cycles.

        IMURaw raw;
        if (IMU_DataReady()) {
            IMU_ReadRaw(&raw);
            IMU_ScaleData(&raw, (IMUData*)&imu_data);
        }

        Filter_Update((const IMUData*)&imu_data, (Attitude*)&attitude);

        // Controller timeout: hold last command if ESP32 silent
        float vel_cmd = cmd_velocity;
        float yaw_cmd = cmd_yaw_rate;
        if (HAL_GetTick() - last_comms_ms > CONTROLLER_TIMEOUT_MS) {
            vel_cmd = 0.0f;
            yaw_cmd = 0.0f;
        }

        // Add gait lean bias to velocity command
        float effective_vel = vel_cmd + gait_output.step_velocity;

        Encoder_Update();
        float vel_l = Encoder_GetVelocity_ms(MOTOR_LEFT);
        float vel_r = Encoder_GetVelocity_ms(MOTOR_RIGHT);

        if (gait_output.balance_active) {
            Balance_Update((const Attitude*)&attitude,
                           vel_l, vel_r,
                           effective_vel, yaw_cmd);
        }
    }

    else if (htim->Instance == TIM7) {
        // ── GAIT LOOP (50Hz) ──────────────────
        // Runs at lower priority than balance loop.
        static const float DT = 1.0f / (float)GAIT_LOOP_HZ;

        // Sync mode with controller input
        RobotMode desired = cmd_walk_mode ? MODE_WALK : MODE_DRIVE;
        if (Gait_GetMode() != desired &&
            Gait_GetMode() != MODE_TRANSITION_TO_WALK &&
            Gait_GetMode() != MODE_TRANSITION_TO_DRIVE) {
            Gait_SetMode(desired);
        }

        Gait_Update(DT, (GaitOutput*)&gait_output);
        Comms_ProcessIncoming();
    }
}

// ── Timer configuration ───────────────────────
static void timer_init() {
    // TIM6: 1kHz balance loop
    // APB1 timer clock = 84MHz
    // Prescaler = 83, ARR = 999 → 84MHz / 84 / 1000 = 1kHz
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 83;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim6);

    // TIM7: 50Hz gait loop
    // Prescaler = 83, ARR = 19999 → 84MHz / 84 / 20000 = 50Hz
    __HAL_RCC_TIM7_CLK_ENABLE();
    htim7.Instance               = TIM7;
    htim7.Init.Prescaler         = 83;
    htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim7.Init.Period            = 19999;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim7);

    // NVIC priorities: balance (TIM6) > gait (TIM7) > main
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);   // highest
    HAL_NVIC_SetPriority(TIM7_IRQn,     1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

// ── Comms callback (called from TIM7 ISR) ─────
// Populated by Comms_ProcessIncoming() in comms.cpp
void Comms_OnControlPacket(float velocity, float yaw, bool walk) {
    cmd_velocity  = velocity;
    cmd_yaw_rate  = yaw;
    cmd_walk_mode = walk;
    last_comms_ms = HAL_GetTick();
}

// ── Main ──────────────────────────────────────
int main(void) {
    HAL_Init();
    // SystemClock_Config() generated by CubeMX — 168MHz on F405

    // ── Peripheral init (CubeMX-generated) ───
    // MX_SPI1_Init(), MX_I2C2_Init(), MX_USART1_Init(),
    // MX_USART2_Init(), MX_GPIO_Init() called here in real project

    // ── Module init ───────────────────────────
    Motor_Init();
    Servo_Init();
    Servo_Park();

    HAL_Delay(100);     // let power rails stabilise

    if (!IMU_Init()) {
        // IMU failed — blink error LED and halt
        while (1) { HAL_Delay(200); }  // replace with LED blink
    }

    // Calibrate at startup — robot must be stationary
    // Flash LED during calibration to indicate not to move
    IMU_Calibrate(500);

    float dt = 1.0f / (float)BALANCE_LOOP_HZ;
    Filter_Init(FILTER_COMPLEMENTARY, dt);
    Balance_Init();
    Gait_Init();
    Encoder_Init();
    Comms_Init();
    Tuning_Init();

    // Start real-time loops
    timer_init();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    // ── Background loop ───────────────────────
    // Only non-real-time tasks here.
    // Real control happens entirely in the ISRs above.
    while (1) {
        // Serial tuning interface
        Tuning_Process();

        // Watchdog: kick hardware watchdog to prevent reset
        // IWDG_KR = 0xAAAA in raw register access
        // or HAL_IWDG_Refresh(&hiwdg);

        // Optional: send telemetry to ESP32 for logging/visualisation
        // Comms_SendTelemetry((const Attitude*)&attitude,
        //                     Encoder_GetVelocity_ms(MOTOR_LEFT),
        //                     Encoder_GetVelocity_ms(MOTOR_RIGHT));

        HAL_Delay(10);
    }
}
