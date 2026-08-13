#pragma once

// ─────────────────────────────────────────────
//  config.h  –  Robot-wide constants & pin map
//  Edit this file to match your hardware.
// ─────────────────────────────────────────────

// ── Robot Physical Parameters ────────────────
// Measure these from your actual robot (metres / kg)
#define ROBOT_MASS_KG           0.8f
#define WHEEL_RADIUS_M          0.0325f   // 65mm diameter TT wheel
#define WHEEL_BASE_M            0.15f     // distance between wheels
#define COM_HEIGHT_M            0.12f     // centre of mass height when standing
#define LEG_UPPER_LENGTH_M      0.08f     // hip to knee
#define LEG_LOWER_LENGTH_M      0.09f     // knee to wheel centre

// ── Control Loop Rates ────────────────────────
#define BALANCE_LOOP_HZ         1000      // inner balance PID
#define GAIT_LOOP_HZ            50        // gait state machine
#define SERVO_UPDATE_HZ         50        // servo PWM refresh
#define COMMS_HZ                100       // UART to ESP32

// ── IMU ───────────────────────────────────────
// ICM-42688 or MPU-6050 on SPI1 / I2C1
#define IMU_USE_SPI             1         // 1 = SPI, 0 = I2C
#define IMU_SPI_PORT            SPI1
#define IMU_CS_PIN              GPIO_PIN_4
#define IMU_CS_PORT             GPIOA
#define IMU_DRDY_PIN            GPIO_PIN_5  // data-ready interrupt
#define IMU_DRDY_PORT           GPIOA
#define IMU_I2C_ADDR            0x68      // if using I2C

// ── Servos ────────────────────────────────────
// PWM: 50Hz, 1000–2000µs pulse width
// TIM3 channels 1-4
#define SERVO_HIP_LEFT_CH       1         // TIM3_CH1 → PA6
#define SERVO_HIP_RIGHT_CH      2         // TIM3_CH2 → PA7
#define SERVO_KNEE_LEFT_CH      3         // TIM3_CH3 → PB0
#define SERVO_KNEE_RIGHT_CH     4         // TIM3_CH4 → PB1

#define SERVO_PWM_MIN_US        1000      // full reverse
#define SERVO_PWM_MID_US        1500      // neutral
#define SERVO_PWM_MAX_US        2000      // full forward

// Mechanical limits (degrees from neutral)
#define HIP_MAX_FORWARD_DEG     45.0f
#define HIP_MAX_BACKWARD_DEG   -45.0f
#define KNEE_MAX_FLEX_DEG       90.0f
#define KNEE_MAX_EXTEND_DEG     0.0f

// ── Motors ────────────────────────────────────
// TB6612FNG or DRV8833 H-bridge
// TIM1 for PWM, GPIO for direction
#define MOTOR_LEFT_PWM_CH       1         // TIM1_CH1 → PA8
#define MOTOR_RIGHT_PWM_CH      2         // TIM1_CH2 → PA9
#define MOTOR_LEFT_DIR_PIN      GPIO_PIN_0
#define MOTOR_LEFT_DIR_PORT     GPIOB
#define MOTOR_RIGHT_DIR_PIN     GPIO_PIN_1  // already used by servo — adjust if needed
#define MOTOR_RIGHT_DIR_PORT    GPIOC
#define MOTOR_PWM_MAX           999       // ARR value → 100% duty

// ── Encoders ─────────────────────────────────
// AS5600 magnetic encoders, I2C2
// Or quadrature on TIM4/TIM5 if using optical encoders
#define ENCODER_USE_MAGNETIC    1         // 1 = AS5600 I2C, 0 = quadrature
#define ENCODER_I2C_PORT        I2C2
#define ENCODER_LEFT_ADDR       0x36      // AS5600 default (add mux if needed)
#define ENCODER_RIGHT_ADDR      0x37      // requires address modification or I2C mux
#define ENCODER_CPR             4096      // AS5600 12-bit = 4096 counts/rev
#define ENCODER_GEAR_RATIO      48.0f     // TT motor gearbox ratio

// ── UART to ESP32 ─────────────────────────────
#define COMMS_UART              USART2
#define COMMS_BAUD              460800
#define COMMS_TX_PIN            GPIO_PIN_2  // PA2
#define COMMS_RX_PIN            GPIO_PIN_3  // PA3
#define COMMS_GPIO_PORT         GPIOA

// ── Serial Tuning Interface ───────────────────
#define TUNING_UART             USART1      // USB-serial via ST-Link VCP
#define TUNING_BAUD             115200

// ── PID Initial Gains ─────────────────────────
// Outer loop: wheel velocity → lean angle setpoint
#define PID_OUTER_KP            2.0f
#define PID_OUTER_KI            0.1f
#define PID_OUTER_KD            0.05f
#define PID_OUTER_MAX_OUTPUT    15.0f     // degrees, max lean command

// Inner loop: lean angle → motor PWM
#define PID_INNER_KP            80.0f
#define PID_INNER_KI            5.0f
#define PID_INNER_KD            12.0f
#define PID_INNER_MAX_OUTPUT    999.0f    // motor PWM counts

// ── LQR Gains (computed offline, update after modelling) ──
// State vector: [theta, theta_dot, x_dot, x]
// Gain vector K = [k1, k2, k3, k4]
#define LQR_K1                  45.2f     // tilt angle gain
#define LQR_K2                  8.1f      // tilt rate gain
#define LQR_K3                  3.5f      // wheel velocity gain
#define LQR_K4                  0.8f      // wheel position gain

// ── Complementary Filter ─────────────────────
#define FILTER_ALPHA            0.98f     // gyro trust (0–1), higher = more gyro

// ── Safety ───────────────────────────────────
#define MAX_TILT_DEG            35.0f     // cut motors beyond this angle
#define WATCHDOG_TIMEOUT_MS     500       // ms before safe shutdown
#define CONTROLLER_TIMEOUT_MS   200       // ms before holding last setpoint
