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
#pragma once
// ─────────────────────────────────────────────
//  imu.h  –  IMU driver interface
//  Supports ICM-42688-P (preferred) and MPU-6050
//  Raw register access over SPI (or I2C fallback)
// ─────────────────────────────────────────────
#include <stdint.h>

// Raw sensor data (straight from registers, unconverted)
struct IMURaw {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x,  gyro_y,  gyro_z;
    int16_t temp;
};

// Scaled sensor data (physical units)
struct IMUData {
    float accel_x, accel_y, accel_z;   // m/s²
    float gyro_x,  gyro_y,  gyro_z;    // rad/s
    float temp_c;                        // °C
};

// Filtered attitude output
struct Attitude {
    float pitch;    // rad — forward/backward tilt (primary balance axis)
    float roll;     // rad — lateral tilt
    float yaw;      // rad — heading (gyro integrated, drifts)
};

bool    IMU_Init(void);
bool    IMU_DataReady(void);       // check DRDY pin
void    IMU_ReadRaw(IMURaw* out);
void    IMU_ScaleData(const IMURaw* raw, IMUData* out);
void    IMU_Calibrate(uint16_t samples);   // average bias at rest
float   IMU_GetPitchBias(void);
float   IMU_GetGyroBias_Y(void);
// ─────────────────────────────────────────────
//  imu.cpp  –  ICM-42688-P / MPU-6050 driver
//
//  Register access is direct — no HAL_Delay,
//  no Arduino Wire. Pure SPI register reads
//  via STM32 HAL SPI in blocking mode for init,
//  then DMA or interrupt-driven for runtime.
//
//  ICM-42688 register map (Bank 0):
//    0x1F = ACCEL_DATA_X1 (high byte, then X0 low)
//    0x25 = GYRO_DATA_X1
//    0x4E = PWR_MGMT0
//    0x50 = GYRO_CONFIG0
//    0x51 = ACCEL_CONFIG0
//    0x76 = WHO_AM_I  → should read 0x47
// ─────────────────────────────────────────────

#include "imu.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <cmath>
#include <cstring>

// ── ICM-42688 Register Definitions ───────────
#define ICM_WHO_AM_I        0x75
#define ICM_WHO_AM_I_VAL    0x47
#define ICM_PWR_MGMT0       0x4E
#define ICM_GYRO_CONFIG0    0x4F
#define ICM_ACCEL_CONFIG0   0x50
#define ICM_GYRO_CONFIG1    0x51
#define ICM_ACCEL_DATA_X1   0x1F   // 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L
#define ICM_GYRO_DATA_X1    0x25   // 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L
#define ICM_TEMP_DATA1      0x1D
#define ICM_INT_CONFIG      0x14
#define ICM_INT_SOURCE0     0x65

// MPU-6050 fallback registers
#define MPU_WHO_AM_I        0x75
#define MPU_WHO_AM_I_VAL    0x68
#define MPU_PWR_MGMT_1      0x6B
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_XOUT_H    0x3B   // 14 bytes: 6 accel + 2 temp + 6 gyro
#define MPU_GYRO_XOUT_H     0x43

// ── Scale factors ─────────────────────────────
// ICM-42688 at ±2g, ±2000dps
#define ACCEL_SCALE  (9.81f / 16384.0f)   // ±2g → LSB/g = 16384
#define GYRO_SCALE   (0.001065f)           // ±2000dps → rad/s per LSB

// ── Internal state ────────────────────────────
static SPI_HandleTypeDef* hspi_imu = nullptr;
static bool use_icm = true;

// Calibration offsets (set during IMU_Calibrate)
static float bias_accel_x = 0, bias_accel_y = 0, bias_accel_z = 0;
static float bias_gyro_x  = 0, bias_gyro_y  = 0, bias_gyro_z  = 0;

// ── SPI helpers ───────────────────────────────
// CS low = select, CS high = deselect
static inline void cs_low()  { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high() { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);   }

// Write one register
static void spi_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(reg & 0x7F), val };   // MSB=0 for write
    cs_low();
    HAL_SPI_Transmit(hspi_imu, buf, 2, 10);
    cs_high();
}

// Read N registers starting at reg
static void spi_read_regs(uint8_t reg, uint8_t* out, uint8_t len) {
    uint8_t tx = reg | 0x80;    // MSB=1 for read
    cs_low();
    HAL_SPI_Transmit(hspi_imu, &tx, 1, 5);
    HAL_SPI_Receive(hspi_imu, out, len, 10);
    cs_high();
}

// ── ICM-42688 Initialisation ──────────────────
static bool icm_init() {
    // Verify chip identity
    uint8_t who = 0;
    spi_read_regs(ICM_WHO_AM_I, &who, 1);
    if (who != ICM_WHO_AM_I_VAL) return false;

    // Soft reset
    spi_write_reg(ICM_PWR_MGMT0, 0x00);
    HAL_Delay(10);

    // Enable accel + gyro in low-noise mode
    // Bits [3:2] = gyro mode: 11 = low noise
    // Bits [1:0] = accel mode: 11 = low noise
    spi_write_reg(ICM_PWR_MGMT0, 0x0F);
    HAL_Delay(1);

    // Gyro: 2000dps, 1kHz ODR
    // Bits [7:5] = FS_SEL: 000 = ±2000dps
    // Bits [3:0] = ODR:   0110 = 1kHz
    spi_write_reg(ICM_GYRO_CONFIG0, 0x06);

    // Accel: ±2g, 1kHz ODR
    spi_write_reg(ICM_ACCEL_CONFIG0, 0x06);

    // Enable data-ready interrupt on INT1 pin
    spi_write_reg(ICM_INT_CONFIG,  0x18);   // INT1 active-high, push-pull
    spi_write_reg(ICM_INT_SOURCE0, 0x08);   // UI_DRDY_INT1_EN

    return true;
}

// ── MPU-6050 Fallback Initialisation ─────────
// Uses I2C2 — only called if ICM init fails
static bool mpu_init() {
    // In a real implementation this would use HAL_I2C_Mem_Write
    // Simplified here — expand with your I2C handle
    return false;   // placeholder
}

// ── Public API ────────────────────────────────
bool IMU_Init(void) {
    // hspi_imu must be initialised before this call
    // In CubeMX-generated code: extern SPI_HandleTypeDef hspi1;
    extern SPI_HandleTypeDef hspi1;
    hspi_imu = &hspi1;

    // CS high initially
    cs_high();
    HAL_Delay(5);

    // Try ICM-42688 first
    if (icm_init()) {
        use_icm = true;
        return true;
    }

    // Fall back to MPU-6050
    use_icm = false;
    return mpu_init();
}

bool IMU_DataReady(void) {
    // Check the DRDY GPIO pin set in config.h
    return HAL_GPIO_ReadPin(IMU_DRDY_PORT, IMU_DRDY_PIN) == GPIO_PIN_SET;
}

void IMU_ReadRaw(IMURaw* out) {
    uint8_t buf[14];    // 6 accel + 2 temp + 6 gyro (contiguous in ICM-42688)

    if (use_icm) {
        // Read 6 accel bytes starting at ACCEL_DATA_X1 (0x1F)
        spi_read_regs(ICM_ACCEL_DATA_X1, buf, 6);
        out->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
        out->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
        out->accel_z = (int16_t)((buf[4] << 8) | buf[5]);

        // Read 6 gyro bytes starting at GYRO_DATA_X1 (0x25)
        spi_read_regs(ICM_GYRO_DATA_X1, buf, 6);
        out->gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
        out->gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
        out->gyro_z = (int16_t)((buf[4] << 8) | buf[5]);

        // Temperature (2 bytes at 0x1D)
        spi_read_regs(ICM_TEMP_DATA1, buf, 2);
        out->temp = (int16_t)((buf[0] << 8) | buf[1]);
    } else {
        // MPU-6050: 14 bytes from ACCEL_XOUT_H
        // Accel[0:5], Temp[6:7], Gyro[8:13]
        spi_read_regs(MPU_ACCEL_XOUT_H, buf, 14);
        out->accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
        out->accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
        out->accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
        out->temp    = (int16_t)((buf[6]  << 8) | buf[7]);
        out->gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
        out->gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
        out->gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);
    }
}

void IMU_ScaleData(const IMURaw* raw, IMUData* out) {
    // Convert to physical units and subtract calibration bias
    out->accel_x = (raw->accel_x * ACCEL_SCALE) - bias_accel_x;
    out->accel_y = (raw->accel_y * ACCEL_SCALE) - bias_accel_y;
    out->accel_z = (raw->accel_z * ACCEL_SCALE) - bias_accel_z;

    out->gyro_x  = (raw->gyro_x  * GYRO_SCALE)  - bias_gyro_x;
    out->gyro_y  = (raw->gyro_y  * GYRO_SCALE)  - bias_gyro_y;
    out->gyro_z  = (raw->gyro_z  * GYRO_SCALE)  - bias_gyro_z;

    // ICM-42688 temp formula: T(°C) = (raw / 132.48) + 25
    // MPU-6050: T(°C) = raw / 340.0 + 36.53
    if (use_icm)
        out->temp_c = (raw->temp / 132.48f) + 25.0f;
    else
        out->temp_c = (raw->temp / 340.0f)  + 36.53f;
}

void IMU_Calibrate(uint16_t samples) {
    // Robot must be stationary and level during this call
    // Average N readings to find sensor bias
    IMURaw raw;
    IMUData scaled;
    float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

    // Temporarily zero biases so ScaleData gives raw physical values
    bias_accel_x = bias_accel_y = bias_accel_z = 0;
    bias_gyro_x  = bias_gyro_y  = bias_gyro_z  = 0;

    for (uint16_t i = 0; i < samples; i++) {
        // Wait for data ready
        while (!IMU_DataReady()) {}
        IMU_ReadRaw(&raw);
        IMU_ScaleData(&raw, &scaled);

        ax += scaled.accel_x;
        ay += scaled.accel_y;
        az += scaled.accel_z;
        gx += scaled.gyro_x;
        gy += scaled.gyro_y;
        gz += scaled.gyro_z;

        HAL_Delay(1);
    }

    float n = (float)samples;

    // Gyro should read 0 when stationary
    bias_gyro_x = gx / n;
    bias_gyro_y = gy / n;
    bias_gyro_z = gz / n;

    // Accel X and Y should read 0 when level
    // Accel Z should read 9.81 (1g) when level — don't bias that out
    bias_accel_x = ax / n;
    bias_accel_y = ay / n;
    bias_accel_z = (az / n) - 9.81f;
}

float IMU_GetPitchBias(void)  { return bias_accel_y; }
float IMU_GetGyroBias_Y(void) { return bias_gyro_y;  }
#pragma once
// ─────────────────────────────────────────────
//  filter.h  –  Attitude filter interface
// ─────────────────────────────────────────────
#include "imu.h"

// Choose which filter to use at runtime
enum FilterType { FILTER_COMPLEMENTARY, FILTER_MADGWICK };

void    Filter_Init(FilterType type, float dt);
void    Filter_Update(const IMUData* imu, Attitude* att);
void    Filter_Reset(void);
void    Filter_SetType(FilterType type);
// ─────────────────────────────────────────────
//  filter.cpp  –  Attitude estimation filters
//
//  Two filters are implemented:
//
//  1. COMPLEMENTARY FILTER
//     Simple, low CPU cost, good enough for
//     most balance robot applications.
//     pitch = alpha*(pitch + gyro*dt)
//           + (1-alpha)*accel_angle
//     Alpha = 0.98 means 98% gyro trust.
//     Gyro is accurate short-term (no drift),
//     accelerometer is accurate long-term (no
//     drift but noisy). The filter fuses both.
//
//  2. MADGWICK FILTER
//     Gradient-descent quaternion filter.
//     Better handling of dynamic acceleration,
//     lower noise floor. Uses more CPU.
//     Recommended once PID is tuned.
//
//  Both output pitch in RADIANS.
//  Positive pitch = robot leaning forward.
// ─────────────────────────────────────────────

#include "filter.h"
#include "config.h"
#include <cmath>

// ── Shared state ──────────────────────────────
static FilterType current_filter;
static float      dt_s;               // sample time in seconds

// ── Complementary filter state ────────────────
static float cf_pitch = 0.0f;
static float cf_roll  = 0.0f;

// ── Madgwick filter state ─────────────────────
// Internal quaternion representing orientation
static float mw_q0 = 1.0f;   // scalar
static float mw_q1 = 0.0f;   // i
static float mw_q2 = 0.0f;   // j
static float mw_q3 = 0.0f;   // k
static float mw_beta = 0.1f; // algorithm gain — higher = faster convergence
                              // but more susceptible to noise.
                              // 0.033 = steady state, 0.1 = fast init

// ── Complementary Filter ──────────────────────
// Pros: trivially simple, very fast, easy to understand
// Cons: susceptible to vibration, needs clean accel signal
static void cf_update(const IMUData* imu, Attitude* att) {

    // Pitch angle from accelerometer (atan2 on X and Z axes)
    // When robot is upright: accel_z ≈ 9.81, accel_x ≈ 0
    // Pitch forward: accel_x increases, accel_z decreases
    float accel_pitch = atan2f(imu->accel_x,
                                sqrtf(imu->accel_y * imu->accel_y
                                    + imu->accel_z * imu->accel_z));

    float accel_roll  = atan2f(imu->accel_y,
                                sqrtf(imu->accel_x * imu->accel_x
                                    + imu->accel_z * imu->accel_z));

    // Complementary fusion:
    // Gyro integration gives short-term accuracy
    // Accelerometer provides long-term correction
    cf_pitch = FILTER_ALPHA * (cf_pitch + imu->gyro_y * dt_s)
             + (1.0f - FILTER_ALPHA) * accel_pitch;

    cf_roll  = FILTER_ALPHA * (cf_roll  + imu->gyro_x * dt_s)
             + (1.0f - FILTER_ALPHA) * accel_roll;

    att->pitch = cf_pitch;
    att->roll  = cf_roll;

    // Yaw: gyro integration only (no absolute reference without magnetometer)
    att->yaw  += imu->gyro_z * dt_s;
}

// ── Madgwick Filter ───────────────────────────
// Reference: Madgwick et al. (2010)
// "An efficient orientation filter for inertial
//  and inertial/magnetic sensor arrays"
//
// Quaternion convention: q = [q0, q1, q2, q3]
// where q0 is scalar, q1-q3 are vector components
static void mw_update(const IMUData* imu, Attitude* att) {
    float q0 = mw_q0, q1 = mw_q1, q2 = mw_q2, q3 = mw_q3;

    float ax = imu->accel_x, ay = imu->accel_y, az = imu->accel_z;
    float gx = imu->gyro_x,  gy = imu->gyro_y,  gz = imu->gyro_z;

    // Normalise accelerometer — if near zero, skip (free fall / no gravity)
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 0.001f) return;
    norm = 1.0f / norm;
    ax *= norm; ay *= norm; az *= norm;

    // Gradient descent step
    // Objective function f = q* ⊗ g ⊗ q - a
    // where g = [0, 0, 0, 1] (gravity in world frame)
    // Partial derivatives of ||f||² w.r.t. quaternion components
    float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

    float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay
             - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
             - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

    // Normalise gradient
    norm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

    // Rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - mw_beta * s0;
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - mw_beta * s1;
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - mw_beta * s2;
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - mw_beta * s3;

    // Integrate to yield quaternion
    mw_q0 = q0 + qDot0 * dt_s;
    mw_q1 = q1 + qDot1 * dt_s;
    mw_q2 = q2 + qDot2 * dt_s;
    mw_q3 = q3 + qDot3 * dt_s;

    // Normalise quaternion
    norm = 1.0f / sqrtf(mw_q0*mw_q0 + mw_q1*mw_q1
                      + mw_q2*mw_q2 + mw_q3*mw_q3);
    mw_q0 *= norm; mw_q1 *= norm;
    mw_q2 *= norm; mw_q3 *= norm;

    // Convert quaternion to Euler angles (pitch and roll for balance)
    // Pitch (rotation about Y axis) — forward/backward tilt
    att->pitch = asinf(2.0f * (mw_q0*mw_q2 - mw_q3*mw_q1));

    // Roll (rotation about X axis) — lateral tilt
    att->roll  = atan2f(2.0f * (mw_q0*mw_q1 + mw_q2*mw_q3),
                        1.0f - 2.0f * (mw_q1*mw_q1 + mw_q2*mw_q2));

    // Yaw (rotation about Z axis) — heading
    att->yaw   = atan2f(2.0f * (mw_q0*mw_q3 + mw_q1*mw_q2),
                        1.0f - 2.0f * (mw_q2*mw_q2 + mw_q3*mw_q3));
}

// ── Public API ────────────────────────────────
void Filter_Init(FilterType type, float dt) {
    current_filter = type;
    dt_s           = dt;
    Filter_Reset();
}

void Filter_Update(const IMUData* imu, Attitude* att) {
    if (current_filter == FILTER_COMPLEMENTARY)
        cf_update(imu, att);
    else
        mw_update(imu, att);
}

void Filter_Reset(void) {
    cf_pitch = cf_roll = 0.0f;
    mw_q0 = 1.0f;
    mw_q1 = mw_q2 = mw_q3 = 0.0f;
}

void Filter_SetType(FilterType type) {
    current_filter = type;
    Filter_Reset();
}
#pragma once
// ─────────────────────────────────────────────
//  servo.h  –  4-channel servo driver
//  Direct TIM3 PWM register access.
//  No Arduino Servo library — all 4 channels
//  update simultaneously via DMA-preloaded CCR.
// ─────────────────────────────────────────────
#include <stdint.h>

// Servo IDs matching config.h channel assignments
enum ServoID {
    SERVO_HIP_LEFT   = 0,
    SERVO_HIP_RIGHT  = 1,
    SERVO_KNEE_LEFT  = 2,
    SERVO_KNEE_RIGHT = 3,
    SERVO_COUNT      = 4
};

void  Servo_Init(void);
void  Servo_SetAngle(ServoID id, float degrees);   // degrees from neutral
void  Servo_SetPulse(ServoID id, uint16_t us);     // raw pulse width µs
void  Servo_SetAll(float hip_l, float hip_r,
                   float knee_l, float knee_r);    // atomic update
void  Servo_Park(void);                            // all to neutral
void  Servo_Disable(void);                         // stop PWM (no torque)
float Servo_GetAngle(ServoID id);
// ─────────────────────────────────────────────
//  servo.cpp  –  Servo PWM via TIM3
//
//  TIM3 is configured for 50Hz PWM (20ms period)
//  with 1µs resolution (1MHz timer clock).
//
//  Timer setup:
//    APB1 clock = 84MHz (STM32F405 default)
//    Prescaler  = 83  → timer clock = 1MHz
//    ARR        = 19999 → period = 20ms = 50Hz
//
//  CCR register value directly = pulse width in µs
//  CCR = 1000 → 1ms  = full reverse
//  CCR = 1500 → 1.5ms = neutral
//  CCR = 2000 → 2ms  = full forward
//
//  All 4 channels share the same timer so all
//  servo pulses start at the same time each
//  cycle — no sequential jitter.
// ─────────────────────────────────────────────

#include "servo.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <algorithm>
#include <cmath>

// ── TIM3 direct register pointers ─────────────
// Using direct register access is faster and
// more explicit than the HAL for runtime updates.
// TIM3 base address from STM32F4 reference manual
#define TIM3_BASE_ADDR   0x40000400UL
#define TIM3_CR1    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x00))
#define TIM3_ARR    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x2C))
#define TIM3_PSC    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x28))
#define TIM3_CCR1   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x34))
#define TIM3_CCR2   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x38))
#define TIM3_CCR3   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x3C))
#define TIM3_CCR4   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x40))
#define TIM3_CCMR1  (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x18))
#define TIM3_CCMR2  (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x1C))
#define TIM3_CCER   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x20))
#define TIM3_EGR    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x14))
#define RCC_APB1ENR (*(volatile uint32_t*)(0x40023840UL))

// ── Internal state ────────────────────────────
static uint16_t pulse_us[SERVO_COUNT];
static float    angle_deg[SERVO_COUNT];

// Pointer array: index → CCR register for fast write
static volatile uint32_t* const ccr[SERVO_COUNT] = {
    &TIM3_CCR1,  // SERVO_HIP_LEFT
    &TIM3_CCR2,  // SERVO_HIP_RIGHT
    &TIM3_CCR3,  // SERVO_KNEE_LEFT
    &TIM3_CCR4   // SERVO_KNEE_RIGHT
};

// ── Angle to pulse width conversion ───────────
// Maps degrees (−90 to +90) to pulse width (1000–2000µs)
// Neutral (0°) = 1500µs
static uint16_t angle_to_us(float deg) {
    // Clamp to servo mechanical limits
    float clamped = fmaxf(-90.0f, fminf(90.0f, deg));
    // Linear map: 0° = 1500µs, ±90° = ±500µs
    float us = SERVO_PWM_MID_US + (clamped / 90.0f) * 500.0f;
    return (uint16_t)fmaxf(SERVO_PWM_MIN_US,
                   fminf(SERVO_PWM_MAX_US, us));
}

// ── Initialise TIM3 for 50Hz PWM ─────────────
void Servo_Init(void) {
    // Enable TIM3 clock via RCC APB1ENR bit 1
    RCC_APB1ENR |= (1 << 1);

    // GPIO setup: PA6, PA7, PB0, PB1 as AF2 (TIM3)
    // This is done in CubeMX — verify your pin config matches config.h

    // Timer configuration
    TIM3_CR1  = 0;           // disable timer during config
    TIM3_PSC  = 83;          // 84MHz / (83+1) = 1MHz timer clock
    TIM3_ARR  = 19999;       // 1MHz / 20000 = 50Hz

    // PWM mode 1 on all 4 channels:
    // CCMR1: CH1 and CH2, CCMR2: CH3 and CH4
    // OC1M/OC2M/OC3M/OC4M = 110 (PWM mode 1)
    // OCxPE = 1 (preload enable — updates take effect at next update event)
    TIM3_CCMR1 = (0x68 << 0)  // CH1: PWM mode 1, preload
               | (0x68 << 8); // CH2: PWM mode 1, preload
    TIM3_CCMR2 = (0x68 << 0)  // CH3: PWM mode 1, preload
               | (0x68 << 8); // CH4: PWM mode 1, preload

    // Enable all 4 capture/compare outputs
    // CC1E, CC2E, CC3E, CC4E bits in CCER
    TIM3_CCER = (1<<0) | (1<<4) | (1<<8) | (1<<12);

    // Set all servos to neutral (1500µs)
    for (int i = 0; i < SERVO_COUNT; i++) {
        pulse_us[i]  = SERVO_PWM_MID_US;
        angle_deg[i] = 0.0f;
        *ccr[i] = SERVO_PWM_MID_US;
    }

    // Generate update event to load preload registers
    TIM3_EGR = 1;

    // Enable timer
    TIM3_CR1 = 1;
}

// ── Public API ────────────────────────────────
void Servo_SetAngle(ServoID id, float degrees) {
    if (id >= SERVO_COUNT) return;

    // Apply mechanical limits from config
    float limited = degrees;
    if (id == SERVO_HIP_LEFT || id == SERVO_HIP_RIGHT) {
        limited = fmaxf(HIP_MAX_BACKWARD_DEG,
                  fminf(HIP_MAX_FORWARD_DEG, degrees));
    } else {
        limited = fmaxf(KNEE_MAX_EXTEND_DEG,
                  fminf(KNEE_MAX_FLEX_DEG, degrees));
    }

    angle_deg[id] = limited;
    pulse_us[id]  = angle_to_us(limited);
    *ccr[id]      = pulse_us[id];   // direct CCR write — takes effect next cycle
}

void Servo_SetPulse(ServoID id, uint16_t us) {
    if (id >= SERVO_COUNT) return;
    uint16_t clamped = (uint16_t)fmaxf(SERVO_PWM_MIN_US,
                                fminf(SERVO_PWM_MAX_US, us));
    pulse_us[id]  = clamped;
    angle_deg[id] = ((float)clamped - SERVO_PWM_MID_US) / 500.0f * 90.0f;
    *ccr[id]      = clamped;
}

void Servo_SetAll(float hip_l, float hip_r, float knee_l, float knee_r) {
    // Write all 4 CCR registers back-to-back for minimum skew
    // All updates take effect at the next PWM cycle start
    TIM3_CCR1 = angle_to_us(fmaxf(HIP_MAX_BACKWARD_DEG,
                             fminf(HIP_MAX_FORWARD_DEG, hip_l)));
    TIM3_CCR2 = angle_to_us(fmaxf(HIP_MAX_BACKWARD_DEG,
                             fminf(HIP_MAX_FORWARD_DEG, hip_r)));
    TIM3_CCR3 = angle_to_us(fmaxf(KNEE_MAX_EXTEND_DEG,
                             fminf(KNEE_MAX_FLEX_DEG, knee_l)));
    TIM3_CCR4 = angle_to_us(fmaxf(KNEE_MAX_EXTEND_DEG,
                             fminf(KNEE_MAX_FLEX_DEG, knee_r)));

    angle_deg[0] = hip_l;  angle_deg[1] = hip_r;
    angle_deg[2] = knee_l; angle_deg[3] = knee_r;
}

void Servo_Park(void) {
    Servo_SetAll(0.0f, 0.0f, 0.0f, 0.0f);
}

void Servo_Disable(void) {
    // Clear CC output enables — servos go slack (no holding torque)
    TIM3_CCER = 0;
}

float Servo_GetAngle(ServoID id) {
    if (id >= SERVO_COUNT) return 0.0f;
    return angle_deg[id];
}
#pragma once
// ─────────────────────────────────────────────
//  motor.h  –  TT motor H-bridge driver
//  encoder.h – AS5600 magnetic encoder reader
// ─────────────────────────────────────────────
#include <stdint.h>

// ── Motor driver ──────────────────────────────
// TB6612FNG or DRV8833 H-bridge
// PWM via TIM1 CH1/CH2 + direction GPIO

enum MotorID { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

void  Motor_Init(void);
void  Motor_Set(MotorID id, float power);   // −1.0 to +1.0
void  Motor_SetBoth(float left, float right);
void  Motor_Brake(void);                    // active braking
void  Motor_Coast(void);                    // free-wheel
float Motor_GetPower(MotorID id);

// ── Encoder reader ────────────────────────────
// AS5600 12-bit magnetic encoder over I2C2
// Reports wheel velocity in rad/s

struct EncoderState {
    int32_t  position;      // cumulative ticks (wraps handled)
    float    velocity_rps;  // wheel revolutions per second
    float    velocity_rads; // wheel angular velocity rad/s
    float    velocity_ms;   // linear wheel velocity m/s
    uint32_t last_update;   // timestamp for velocity calculation
};

void  Encoder_Init(void);
void  Encoder_Update(void);                      // call at control loop rate
const EncoderState* Encoder_Get(MotorID id);
float Encoder_GetVelocity_ms(MotorID id);        // m/s convenience
void  Encoder_Reset(MotorID id);
// ─────────────────────────────────────────────
//  motor.cpp + encoder.cpp  –  Motor and encoder
//
//  Motor: TIM1 CH1/CH2 PWM + direction GPIO
//  TIM1 is an advanced timer on APB2 (168MHz on F405)
//  Prescaler = 0 → 168MHz, ARR = 999 → 168kHz PWM
//  High PWM frequency reduces motor noise and
//  improves low-speed control.
//
//  Encoder: AS5600 over I2C2
//  AS5600 outputs 12-bit absolute angle (0–4095)
//  We track multi-turn position and differentiate
//  for velocity. Both encoders need either separate
//  I2C buses or an I2C mux (TCA9548A).
//
//  Velocity is computed with a simple finite
//  difference and smoothed with a low-pass filter.
// ─────────────────────────────────────────────

#include "motor.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <cmath>
#include <cstdlib>

// ── TIM1 register access ──────────────────────
#define TIM1_BASE_ADDR   0x40010000UL
#define TIM1_CR1    (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x00))
#define TIM1_ARR    (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x2C))
#define TIM1_PSC    (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x28))
#define TIM1_CCR1   (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x34))
#define TIM1_CCR2   (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x38))
#define TIM1_CCMR1  (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x18))
#define TIM1_CCER   (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x20))
#define TIM1_BDTR   (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x44))
#define TIM1_EGR    (*(volatile uint32_t*)(TIM1_BASE_ADDR + 0x14))
#define RCC_APB2ENR (*(volatile uint32_t*)(0x40023844UL))

// ── AS5600 register definitions ───────────────
#define AS5600_ANGLE_H   0x0E    // high byte of raw angle
#define AS5600_ANGLE_L   0x0F    // low byte (12-bit result: bits [11:0])
#define AS5600_STATUS    0x0B    // MD=1 magnet detected, ML=1 too weak, MH=1 too strong

// ── I2C2 handle (from CubeMX) ─────────────────
extern I2C_HandleTypeDef hi2c2;

// ── Internal state ────────────────────────────
static float motor_power[2] = {0.0f, 0.0f};

static EncoderState enc[2];
static uint16_t     enc_prev_raw[2];    // previous 12-bit angle reading
static int32_t      enc_turns[2];       // full-turn counter for unwrapping

// Low-pass filter for velocity
// alpha close to 1 = more smoothing, more lag
#define VEL_LPF_ALPHA   0.15f
static float vel_filtered[2];

// I2C addresses for left/right encoders
// AS5600 has fixed 0x36 address — use I2C mux or separate buses
static const uint8_t enc_addr[2] = {
    ENCODER_LEFT_ADDR  << 1,
    ENCODER_RIGHT_ADDR << 1
};

// ── Motor helpers ─────────────────────────────
static void set_motor_pwm(MotorID id, float power) {
    // power: -1.0 to +1.0
    // Positive = forward, negative = reverse
    float clamped = fmaxf(-1.0f, fminf(1.0f, power));
    motor_power[id] = clamped;

    uint32_t pwm_val = (uint32_t)(fabsf(clamped) * MOTOR_PWM_MAX);
    bool     forward = (clamped >= 0.0f);

    if (id == MOTOR_LEFT) {
        HAL_GPIO_WritePin(MOTOR_LEFT_DIR_PORT, MOTOR_LEFT_DIR_PIN,
                          forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
        TIM1_CCR1 = pwm_val;
    } else {
        // Right motor is mirrored — invert direction
        HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN,
                          forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
        TIM1_CCR2 = pwm_val;
    }
}

// ── Motor public API ──────────────────────────
void Motor_Init(void) {
    // Enable TIM1 on APB2
    RCC_APB2ENR |= (1 << 0);

    // Configure TIM1 for high-frequency PWM
    TIM1_CR1   = 0;
    TIM1_PSC   = 0;       // 168MHz / 1 = 168MHz timer clock
    TIM1_ARR   = 999;     // 168MHz / 1000 = 168kHz PWM
    TIM1_CCMR1 = (0x68 << 0) | (0x68 << 8);  // PWM mode 1 on CH1, CH2
    TIM1_CCER  = (1<<0) | (1<<4);             // Enable CC1 and CC2
    TIM1_BDTR  = (1<<15);  // MOE = main output enable (required for TIM1)
    TIM1_CCR1  = 0;
    TIM1_CCR2  = 0;
    TIM1_EGR   = 1;        // load preload registers
    TIM1_CR1   = 1;        // enable timer

    // Init direction GPIO as output (done via CubeMX in real project)
    Motor_Coast();
}

void Motor_Set(MotorID id, float power) {
    set_motor_pwm(id, power);
}

void Motor_SetBoth(float left, float right) {
    set_motor_pwm(MOTOR_LEFT,  left);
    set_motor_pwm(MOTOR_RIGHT, right);
}

void Motor_Brake(void) {
    // Active braking: both direction pins high, PWM = max
    HAL_GPIO_WritePin(MOTOR_LEFT_DIR_PORT,  MOTOR_LEFT_DIR_PIN,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_RIGHT_DIR_PORT, MOTOR_RIGHT_DIR_PIN, GPIO_PIN_SET);
    TIM1_CCR1 = MOTOR_PWM_MAX;
    TIM1_CCR2 = MOTOR_PWM_MAX;
    motor_power[0] = motor_power[1] = 0.0f;
}

void Motor_Coast(void) {
    TIM1_CCR1 = 0;
    TIM1_CCR2 = 0;
    motor_power[0] = motor_power[1] = 0.0f;
}

float Motor_GetPower(MotorID id) {
    return motor_power[id];
}

// ── Encoder helpers ───────────────────────────
// Read 12-bit angle from AS5600 over I2C
static uint16_t read_as5600_angle(MotorID id) {
    uint8_t buf[2];
    // Read 2 bytes from ANGLE_H register
    HAL_I2C_Mem_Read(&hi2c2,
                     enc_addr[id],
                     AS5600_ANGLE_H,
                     I2C_MEMADD_SIZE_8BIT,
                     buf, 2,
                     5);    // 5ms timeout
    return (uint16_t)(((buf[0] & 0x0F) << 8) | buf[1]);
}

// Unwrap 12-bit angle into continuous multi-turn position
// Handles wrap at 0/4095 boundary
static int16_t unwrap_delta(uint16_t prev, uint16_t curr) {
    int16_t delta = (int16_t)curr - (int16_t)prev;
    // If delta > half range, we wrapped backwards
    if (delta >  (ENCODER_CPR / 2)) delta -= ENCODER_CPR;
    if (delta < -(ENCODER_CPR / 2)) delta += ENCODER_CPR;
    return delta;
}

// ── Encoder public API ────────────────────────
void Encoder_Init(void) {
    for (int i = 0; i < 2; i++) {
        enc[i].position     = 0;
        enc[i].velocity_rps = 0.0f;
        enc[i].velocity_rads = 0.0f;
        enc[i].velocity_ms  = 0.0f;
        enc[i].last_update  = HAL_GetTick();
        enc_prev_raw[i]     = read_as5600_angle((MotorID)i);
        enc_turns[i]        = 0;
        vel_filtered[i]     = 0.0f;
    }
}

void Encoder_Update(void) {
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < 2; i++) {
        uint16_t raw   = read_as5600_angle((MotorID)i);
        int16_t  delta = unwrap_delta(enc_prev_raw[i], raw);

        // Accumulate position in raw encoder ticks
        enc[i].position += delta;
        enc_prev_raw[i]  = raw;

        // Compute velocity: ticks per ms → convert to useful units
        float dt_ms = (float)(now - enc[i].last_update);
        if (dt_ms > 0.0f) {
            float raw_vel_rps = ((float)delta / ENCODER_CPR)
                              / (dt_ms * 0.001f)
                              / ENCODER_GEAR_RATIO;

            // Low-pass filter to reduce noise
            vel_filtered[i] = VEL_LPF_ALPHA * raw_vel_rps
                             + (1.0f - VEL_LPF_ALPHA) * vel_filtered[i];

            enc[i].velocity_rps  = vel_filtered[i];
            enc[i].velocity_rads = vel_filtered[i] * 2.0f * 3.14159f;
            enc[i].velocity_ms   = enc[i].velocity_rads * WHEEL_RADIUS_M;
        }

        enc[i].last_update = now;
    }
}

const EncoderState* Encoder_Get(MotorID id) {
    return &enc[id];
}

float Encoder_GetVelocity_ms(MotorID id) {
    return enc[id].velocity_ms;
}

void Encoder_Reset(MotorID id) {
    enc[id].position    = 0;
    enc_turns[id]       = 0;
    vel_filtered[id]    = 0.0f;
    enc[id].velocity_rps = enc[id].velocity_rads = enc[id].velocity_ms = 0.0f;
}
#pragma once
// ─────────────────────────────────────────────
//  balance.h  –  Inverted pendulum controller
//  Implements Cascade PID and LQR
//  Switch between them via Balance_SetMode()
// ─────────────────────────────────────────────
#include "filter.h"
#include "motor.h"

enum BalanceMode { BALANCE_PID, BALANCE_LQR };

// PID gains (modifiable at runtime via tuning interface)
struct PIDGains {
    float kp, ki, kd;
    float max_output;
    float max_integral;     // anti-windup clamp
};

void  Balance_Init(void);
void  Balance_Update(const Attitude* att,
                     float vel_left_ms, float vel_right_ms,
                     float desired_vel_ms, float desired_yaw_rate);
void  Balance_SetMode(BalanceMode mode);
void  Balance_SetGains(const PIDGains* inner, const PIDGains* outer);
void  Balance_GetGains(PIDGains* inner_out, PIDGains* outer_out);
void  Balance_Reset(void);
bool  Balance_IsStable(void);    // false if tilt > MAX_TILT_DEG
float Balance_GetLeanSetpoint(void);
// ─────────────────────────────────────────────
//  balance.cpp  –  Balance controller
//
//  CASCADE PID ARCHITECTURE:
//  ┌─────────────────────────────────────────┐
//  │ OUTER LOOP (50–100Hz)                   │
//  │  Input:  desired wheel velocity         │
//  │  Output: target lean angle (setpoint)   │
//  │  Tuning: slow gains, small output range │
//  └─────────────┬───────────────────────────┘
//                │ lean_setpoint_rad
//  ┌─────────────▼───────────────────────────┐
//  │ INNER LOOP (1000Hz)                     │
//  │  Input:  actual pitch vs lean_setpoint  │
//  │  Output: motor PWM command              │
//  │  Tuning: fast gains, full output range  │
//  └─────────────────────────────────────────┘
//
//  The outer loop converts "I want to go 0.3 m/s"
//  into "lean forward 3 degrees". The inner loop
//  then drives the motors to achieve and hold that
//  lean angle. This cascade structure separates
//  velocity control from balance stabilisation.
//
//  LQR ARCHITECTURE:
//  ┌─────────────────────────────────────────┐
//  │ State vector x = [θ, θ̇, ẋ, x]         │
//  │  θ  = pitch angle (rad)                 │
//  │  θ̇  = pitch rate (rad/s)               │
//  │  ẋ  = wheel velocity (m/s)              │
//  │  x  = wheel position (m)                │
//  │                                         │
//  │ Control: u = -K·x + K_vel·v_desired     │
//  │ K = [k1, k2, k3, k4] from Riccati eq.  │
//  │                                         │
//  │ K is computed offline using MATLAB/     │
//  │ Python (scipy.linalg.solve_continuous_are)│
//  │ Update LQR_K1–K4 in config.h            │
//  └─────────────────────────────────────────┘
//
//  HOW TO COMPUTE LQR GAINS:
//  1. Measure your robot: mass, wheel radius, CoM height
//  2. Build linearised state-space model (A, B matrices)
//  3. Choose Q (state penalty) and R (control penalty)
//  4. Solve: K = lqr(A, B, Q, R) in Python/MATLAB
//  5. Update config.h with the K values
//  A worked example is in the comments below.
// ─────────────────────────────────────────────

#include "balance.h"
#include "motor.h"
#include "config.h"
#include <cmath>
#include <algorithm>

// ── Internal types ────────────────────────────
struct PIDState {
    float integral;
    float prev_error;
    float prev_deriv;   // for derivative low-pass
};

// ── Internal state ────────────────────────────
static BalanceMode   mode;
static PIDGains      gains_inner, gains_outer;
static PIDState      state_inner, state_outer;

// LQR state
static float         wheel_pos_m = 0.0f;   // integrated position
static float         wheel_vel_m = 0.0f;   // from encoders

static bool          is_stable   = true;
static float         lean_setpt  = 0.0f;   // rad

// dt for inner loop (1 / BALANCE_LOOP_HZ)
static const float DT_INNER = 1.0f / (float)BALANCE_LOOP_HZ;
static const float DT_OUTER = 1.0f / (float)GAIT_LOOP_HZ;

// Derivative low-pass coefficient (reduces noise amplification)
// Higher = more smoothing, more lag
#define DERIV_LPF   0.7f

// ── PID helper ────────────────────────────────
static float pid_compute(const PIDGains* g, PIDState* s,
                         float error, float dt) {
    // Proportional
    float p = g->kp * error;

    // Integral with anti-windup clamp
    s->integral += error * dt;
    s->integral  = fmaxf(-g->max_integral,
                   fminf( g->max_integral, s->integral));
    float i = g->ki * s->integral;

    // Derivative with low-pass filter
    float raw_d = (error - s->prev_error) / dt;
    float d_filt = DERIV_LPF * s->prev_deriv
                 + (1.0f - DERIV_LPF) * raw_d;
    s->prev_deriv  = d_filt;
    s->prev_error  = error;
    float d = g->kd * d_filt;

    float out = p + i + d;
    return fmaxf(-g->max_output, fminf(g->max_output, out));
}

// ── Cascade PID update ────────────────────────
static void update_pid(const Attitude* att,
                       float vel_avg_ms,
                       float desired_vel_ms,
                       float desired_yaw_ms) {
    // OUTER LOOP: velocity error → lean angle setpoint
    // Runs at GAIT_LOOP_HZ but we call it every inner loop tick
    // and just update when enough time has passed (or call outer
    // loop from a separate timer — see main.cpp)
    float vel_error = desired_vel_ms - vel_avg_ms;
    lean_setpt = pid_compute(&gains_outer, &state_outer,
                              vel_error, DT_OUTER);
    // lean_setpt is now in radians (e.g. 0.05 rad = ~3°)

    // INNER LOOP: pitch error → motor command
    float pitch_error = att->pitch - lean_setpt;
    float motor_cmd   = pid_compute(&gains_inner, &state_inner,
                                     pitch_error, DT_INNER);

    // Yaw: differential drive — add/subtract from each motor
    // Yaw command comes from controller turn stick
    float yaw_diff = desired_yaw_ms * (float)MOTOR_PWM_MAX;

    // Normalise motor_cmd to -1.0 to 1.0 range
    float base = motor_cmd / (float)MOTOR_PWM_MAX;
    float left  = base + yaw_diff / (float)MOTOR_PWM_MAX;
    float right = base - yaw_diff / (float)MOTOR_PWM_MAX;

    // Clamp and apply
    Motor_SetBoth(fmaxf(-1.0f, fminf(1.0f, left)),
                  fmaxf(-1.0f, fminf(1.0f, right)));
}

// ── LQR update ────────────────────────────────
//
// Physical model of wheeled inverted pendulum:
//
//  State: x = [θ, θ̇, ẋ_w, x_w]
//   θ    = body tilt angle (rad), positive = forward lean
//   θ̇    = body tilt rate (rad/s)
//   ẋ_w  = wheel linear velocity (m/s)
//   x_w  = wheel position (m), integrated
//
//  System matrices (linearised about upright θ=0):
//   Derived from Lagrangian mechanics for inverted pendulum on wheels
//   M = total mass, m = body mass, l = CoM height, I = body inertia
//   r = wheel radius, g = 9.81
//
//  Typical LQR gain computation in Python:
//   import numpy as np
//   from scipy.linalg import solve_continuous_are
//
//   M=0.8; m=0.6; l=0.12; I=0.002; r=0.0325; g=9.81
//   den = M*I + m*I + M*m*l*l
//   A = np.array([[0,1,0,0],
//                 [m*g*l*(M+m)/den, 0, 0, 0],
//                 [-m*m*g*l*l/den, 0, 0, 0],
//                 [0,0,1,0]])
//   B = np.array([[0],
//                 [-(I+m*l*l)/den],
//                 [(I+m*l*l+m*l*r)/den/r],
//                 [0]])
//   Q = np.diag([100, 1, 10, 1])   # penalise tilt most
//   R = np.array([[0.01]])
//   P = solve_continuous_are(A, B, Q, R)
//   K = np.linalg.inv(R) @ B.T @ P
//   print(K)  # → update LQR_K1..K4 in config.h
//
static void update_lqr(const Attitude* att,
                       float vel_avg_ms,
                       float desired_vel_ms,
                       float desired_yaw_rate) {
    // Update integrated wheel position
    wheel_pos_m += vel_avg_ms * DT_INNER;
    wheel_vel_m  = vel_avg_ms;

    // State vector
    float theta     = att->pitch;
    float theta_dot = att->yaw;      // gyro Y axis = pitch rate
    // (depends on your IMU orientation — verify in hardware)
    float x_dot     = wheel_vel_m;
    float x_pos     = wheel_pos_m;

    // Velocity tracking: add integral term for desired velocity
    // x_pos_setpoint integrates desired velocity over time
    static float x_pos_setpoint = 0.0f;
    x_pos_setpoint += desired_vel_ms * DT_INNER;

    // LQR control law: u = -K * (x - x_ref)
    // x_ref = [0, 0, desired_vel, x_pos_setpoint]
    float u = -(  LQR_K1 * (theta   - 0.0f)
                + LQR_K2 * (theta_dot - 0.0f)
                + LQR_K3 * (x_dot   - desired_vel_ms)
                + LQR_K4 * (x_pos   - x_pos_setpoint));

    // Normalise u to ±1.0 range
    float base  = fmaxf(-1.0f, fminf(1.0f, u / 50.0f));
    float yaw   = desired_yaw_rate * 0.3f;

    Motor_SetBoth(fmaxf(-1.0f, fminf(1.0f, base + yaw)),
                  fmaxf(-1.0f, fminf(1.0f, base - yaw)));
}

// ── Public API ────────────────────────────────
void Balance_Init(void) {
    mode = BALANCE_PID;

    gains_inner = { PID_INNER_KP, PID_INNER_KI, PID_INNER_KD,
                    PID_INNER_MAX_OUTPUT, PID_INNER_MAX_OUTPUT * 0.5f };
    gains_outer = { PID_OUTER_KP, PID_OUTER_KI, PID_OUTER_KD,
                    PID_OUTER_MAX_OUTPUT * 3.14159f / 180.0f,  // convert deg to rad
                    PID_OUTER_MAX_OUTPUT * 3.14159f / 180.0f * 0.5f };

    Balance_Reset();
}

void Balance_Update(const Attitude* att,
                    float vel_left_ms, float vel_right_ms,
                    float desired_vel_ms, float desired_yaw_rate) {

    float vel_avg = (vel_left_ms + vel_right_ms) * 0.5f;

    // Safety check — cut motors if tilt is too large
    float tilt_deg = fabsf(att->pitch) * (180.0f / 3.14159f);
    if (tilt_deg > MAX_TILT_DEG) {
        is_stable = false;
        Motor_Coast();
        return;
    }
    is_stable = true;

    if (mode == BALANCE_PID)
        update_pid(att, vel_avg, desired_vel_ms, desired_yaw_rate);
    else
        update_lqr(att, vel_avg, desired_vel_ms, desired_yaw_rate);
}

void Balance_SetMode(BalanceMode m) {
    mode = m;
    Balance_Reset();
    wheel_pos_m = 0.0f;
}

void Balance_SetGains(const PIDGains* inner, const PIDGains* outer) {
    if (inner) gains_inner = *inner;
    if (outer) gains_outer = *outer;
}

void Balance_GetGains(PIDGains* inner_out, PIDGains* outer_out) {
    if (inner_out) *inner_out = gains_inner;
    if (outer_out) *outer_out = gains_outer;
}

void Balance_Reset(void) {
    state_inner = {0.0f, 0.0f, 0.0f};
    state_outer = {0.0f, 0.0f, 0.0f};
    lean_setpt  = 0.0f;
    is_stable   = true;
}

bool  Balance_IsStable(void)       { return is_stable;  }
float Balance_GetLeanSetpoint(void){ return lean_setpt; }
#pragma once
// ─────────────────────────────────────────────
//  gait.h  –  Finite State Machine gait sequencer
//  Controls hip and knee servos during walk mode.
//  Outputs lean angle bias to balance controller.
// ─────────────────────────────────────────────
#include <stdint.h>

// Robot operating modes
enum RobotMode {
    MODE_IDLE,          // standing still, balance active
    MODE_DRIVE,         // wheeled drive, balance active, legs straight
    MODE_WALK,          // gait active, balance modified
    MODE_TRANSITION_TO_WALK,
    MODE_TRANSITION_TO_DRIVE,
    MODE_SAFE_SIT       // emergency: lower to ground
};

// Gait states (sub-states of MODE_WALK)
enum GaitState {
    GAIT_STAND,
    GAIT_WEIGHT_SHIFT_LEFT,
    GAIT_SWING_RIGHT,
    GAIT_PLANT_RIGHT,
    GAIT_WEIGHT_SHIFT_RIGHT,
    GAIT_SWING_LEFT,
    GAIT_PLANT_LEFT,
};

// Per-state joint targets
struct LegPose {
    float hip_left_deg;
    float hip_right_deg;
    float knee_left_deg;
    float knee_right_deg;
};

// Output from gait to balance controller
struct GaitOutput {
    LegPose  target_pose;    // desired joint angles
    float    lean_bias_rad;  // additional lean request to balance controller
    float    step_velocity;  // desired forward velocity for this step phase
    bool     balance_active; // false during sit/stand transitions
};

void        Gait_Init(void);
void        Gait_Update(float dt, GaitOutput* out);
void        Gait_SetMode(RobotMode mode);
RobotMode   Gait_GetMode(void);
GaitState   Gait_GetState(void);
void        Gait_SetWalkSpeed(float normalised);   // 0.0 to 1.0
void        Gait_SetStepHeight(float deg);         // knee lift angle
const char* Gait_GetModeName(void);
// ─────────────────────────────────────────────
//  gait.cpp  –  FSM gait sequencer
//
//  GAIT CYCLE (rolling biped walk):
//
//  STAND ──► WEIGHT_SHIFT_LEFT ──► SWING_RIGHT ──► PLANT_RIGHT
//                                                       │
//  STAND ◄── WEIGHT_SHIFT_LEFT ◄── SWING_LEFT  ◄── WEIGHT_SHIFT_RIGHT
//
//  During WEIGHT_SHIFT: lean slightly toward stance leg so
//  swing leg is unloaded (gravity does the work)
//
//  During SWING: hip forward + knee lift on swing leg
//  During PLANT: knee extends, foot contacts ground
//
//  TRAPEZOIDAL INTERPOLATION:
//  Instead of snapping to target angles, each joint
//  ramps up to speed, holds, then ramps down.
//  This prevents jerk and reduces servo wear.
//
//        target ────────────────
//              /                \
//             /                  \
//  current ──/                    \── next target
//          ramp up   hold    ramp down
//
//  The ramp rate (deg/s) is configurable per joint.
// ─────────────────────────────────────────────

#include "gait.h"
#include "servo.h"
#include "config.h"
#include <cmath>
#include <cstring>

// ── Interpolator ──────────────────────────────
// Tracks a single joint through a trapezoidal
// velocity profile between current and target angle
struct Interpolator {
    float current;      // current angle (deg)
    float target;       // target angle (deg)
    float velocity;     // current movement velocity (deg/s)
    float max_vel;      // maximum velocity (deg/s)
    float accel;        // acceleration (deg/s²)

    bool at_target() const {
        return fabsf(current - target) < 0.5f;
    }
};

// Step the interpolator by dt seconds
// Returns current angle after this step
static float interp_step(Interpolator* ip, float dt) {
    float error = ip->target - ip->current;
    float dist  = fabsf(error);
    if (dist < 0.1f) {
        ip->current  = ip->target;
        ip->velocity = 0.0f;
        return ip->current;
    }
    float dir = (error > 0.0f) ? 1.0f : -1.0f;

    // Braking distance at current velocity: v²/(2a)
    float brake_dist = (ip->velocity * ip->velocity) / (2.0f * ip->accel);

    if (dist <= brake_dist) {
        // Decelerate
        ip->velocity -= ip->accel * dt;
        if (ip->velocity < 0.0f) ip->velocity = 0.0f;
    } else {
        // Accelerate or hold max
        ip->velocity += ip->accel * dt;
        if (ip->velocity > ip->max_vel) ip->velocity = ip->max_vel;
    }

    ip->current += dir * ip->velocity * dt;
    // Clamp — don't overshoot
    if ((dir > 0 && ip->current > ip->target) ||
        (dir < 0 && ip->current < ip->target)) {
        ip->current  = ip->target;
        ip->velocity = 0.0f;
    }
    return ip->current;
}

// ── Gait state machine ────────────────────────
static RobotMode   robot_mode   = MODE_IDLE;
static GaitState   gait_state   = GAIT_STAND;
static float       state_timer  = 0.0f;     // time in current state
static float       walk_speed   = 0.5f;     // 0–1
static float       step_height  = 30.0f;    // knee lift degrees

// One interpolator per servo
static Interpolator interp[SERVO_COUNT];

// ── Gait pose definitions ─────────────────────
// These are the target joint angles for each gait phase.
// All angles in degrees from neutral (0).
// Positive hip = forward, positive knee = flexed.
//
// Adjust these to match your robot's geometry.

static const LegPose POSE_STAND = {
    .hip_left_deg   =  0.0f,
    .hip_right_deg  =  0.0f,
    .knee_left_deg  =  0.0f,
    .knee_right_deg =  0.0f
};

static const LegPose POSE_WEIGHT_LEFT = {
    // Lean body left by adjusting both hips slightly
    // Left leg becomes stance, right leg unloaded
    .hip_left_deg   =  0.0f,
    .hip_right_deg  =  0.0f,
    .knee_left_deg  =  5.0f,   // slight squat on stance leg
    .knee_right_deg =  0.0f
};

static const LegPose POSE_SWING_RIGHT = {
    // Right leg swings forward: hip forward + knee lift
    .hip_left_deg   =  0.0f,
    .hip_right_deg  = 20.0f,   // swing forward
    .knee_left_deg  =  5.0f,
    .knee_right_deg = 30.0f    // lift knee (set by step_height)
};

static const LegPose POSE_PLANT_RIGHT = {
    // Right foot plants: knee extends
    .hip_left_deg   =  0.0f,
    .hip_right_deg  = 20.0f,
    .knee_left_deg  =  5.0f,
    .knee_right_deg =  0.0f    // extend — wheel contacts ground
};

static const LegPose POSE_WEIGHT_RIGHT = {
    .hip_left_deg   =  0.0f,
    .hip_right_deg  =  0.0f,
    .knee_left_deg  =  0.0f,
    .knee_right_deg =  5.0f
};

static const LegPose POSE_SWING_LEFT = {
    .hip_left_deg   = 20.0f,
    .hip_right_deg  =  0.0f,
    .knee_left_deg  = 30.0f,
    .knee_right_deg =  5.0f
};

static const LegPose POSE_PLANT_LEFT = {
    .hip_left_deg   = 20.0f,
    .hip_right_deg  =  0.0f,
    .knee_left_deg  =  0.0f,
    .knee_right_deg =  5.0f
};

// ── Set interpolator targets from pose ────────
static void set_targets(const LegPose* p) {
    interp[SERVO_HIP_LEFT ].target = p->hip_left_deg;
    interp[SERVO_HIP_RIGHT].target = p->hip_right_deg;
    interp[SERVO_KNEE_LEFT].target = p->knee_left_deg;
    interp[SERVO_KNEE_RIGHT].target = p->knee_right_deg;
}

// Check if all joints have reached their targets
static bool all_at_target() {
    for (int i = 0; i < SERVO_COUNT; i++)
        if (!interp[i].at_target()) return false;
    return true;
}

// State durations (seconds) — scale with walk_speed
static float state_duration(GaitState s) {
    float base_dur;
    switch (s) {
        case GAIT_WEIGHT_SHIFT_LEFT:
        case GAIT_WEIGHT_SHIFT_RIGHT: base_dur = 0.4f; break;
        case GAIT_SWING_RIGHT:
        case GAIT_SWING_LEFT:         base_dur = 0.5f; break;
        case GAIT_PLANT_RIGHT:
        case GAIT_PLANT_LEFT:         base_dur = 0.2f; break;
        default:                       base_dur = 0.5f; break;
    }
    // Faster walk_speed → shorter state durations
    return base_dur * (1.0f - 0.5f * walk_speed);
}

// Advance to next gait state
static void advance_state() {
    switch (gait_state) {
        case GAIT_STAND:
            gait_state = GAIT_WEIGHT_SHIFT_LEFT;
            set_targets(&POSE_WEIGHT_LEFT);
            break;
        case GAIT_WEIGHT_SHIFT_LEFT:
            gait_state = GAIT_SWING_RIGHT;
            // Apply step_height to knee
            interp[SERVO_KNEE_RIGHT].target = step_height;
            set_targets(&POSE_SWING_RIGHT);
            interp[SERVO_KNEE_RIGHT].target = step_height; // override
            break;
        case GAIT_SWING_RIGHT:
            gait_state = GAIT_PLANT_RIGHT;
            set_targets(&POSE_PLANT_RIGHT);
            break;
        case GAIT_PLANT_RIGHT:
            gait_state = GAIT_WEIGHT_SHIFT_RIGHT;
            set_targets(&POSE_WEIGHT_RIGHT);
            break;
        case GAIT_WEIGHT_SHIFT_RIGHT:
            gait_state = GAIT_SWING_LEFT;
            set_targets(&POSE_SWING_LEFT);
            interp[SERVO_KNEE_LEFT].target = step_height;
            break;
        case GAIT_SWING_LEFT:
            gait_state = GAIT_PLANT_LEFT;
            set_targets(&POSE_PLANT_LEFT);
            break;
        case GAIT_PLANT_LEFT:
            gait_state = GAIT_WEIGHT_SHIFT_LEFT;
            set_targets(&POSE_WEIGHT_LEFT);
            break;
    }
    state_timer = 0.0f;
}

// ── Public API ────────────────────────────────
void Gait_Init(void) {
    robot_mode  = MODE_IDLE;
    gait_state  = GAIT_STAND;
    state_timer = 0.0f;

    // Init all interpolators
    for (int i = 0; i < SERVO_COUNT; i++) {
        interp[i].current  = 0.0f;
        interp[i].target   = 0.0f;
        interp[i].velocity = 0.0f;
        interp[i].max_vel  = 120.0f;  // deg/s
        interp[i].accel    = 300.0f;  // deg/s²
    }
}

void Gait_Update(float dt, GaitOutput* out) {
    // Step all interpolators
    float angles[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
        angles[i] = interp_step(&interp[i], dt);

    // Write to servos
    Servo_SetAll(angles[SERVO_HIP_LEFT],  angles[SERVO_HIP_RIGHT],
                 angles[SERVO_KNEE_LEFT], angles[SERVO_KNEE_RIGHT]);

    // Fill output struct
    out->target_pose.hip_left_deg   = angles[SERVO_HIP_LEFT];
    out->target_pose.hip_right_deg  = angles[SERVO_HIP_RIGHT];
    out->target_pose.knee_left_deg  = angles[SERVO_KNEE_LEFT];
    out->target_pose.knee_right_deg = angles[SERVO_KNEE_RIGHT];
    out->balance_active = (robot_mode != MODE_SAFE_SIT);

    state_timer += dt;

    switch (robot_mode) {
        case MODE_IDLE:
        case MODE_DRIVE:
            // Servos park at stand pose
            set_targets(&POSE_STAND);
            out->lean_bias_rad  = 0.0f;
            out->step_velocity  = 0.0f;
            break;

        case MODE_WALK:
            // Advance gait state when duration elapsed and joints settled
            if (state_timer >= state_duration(gait_state) && all_at_target())
                advance_state();

            // Lean bias during weight shift phases
            if (gait_state == GAIT_WEIGHT_SHIFT_LEFT)
                out->lean_bias_rad = -0.03f;  // lean left slightly
            else if (gait_state == GAIT_WEIGHT_SHIFT_RIGHT)
                out->lean_bias_rad =  0.03f;
            else
                out->lean_bias_rad =  0.0f;

            out->step_velocity = walk_speed * 0.3f;  // m/s
            break;

        case MODE_TRANSITION_TO_WALK:
            // Intermediate: stand up fully then switch to walk
            set_targets(&POSE_STAND);
            if (all_at_target()) {
                robot_mode = MODE_WALK;
                gait_state = GAIT_STAND;
            }
            out->lean_bias_rad = 0.0f;
            out->step_velocity = 0.0f;
            break;

        case MODE_TRANSITION_TO_DRIVE:
            set_targets(&POSE_STAND);
            if (all_at_target()) robot_mode = MODE_DRIVE;
            out->lean_bias_rad = 0.0f;
            out->step_velocity = 0.0f;
            break;

        case MODE_SAFE_SIT:
            // Lower to ground — reduce knee angle gradually
            for (int i = 0; i < SERVO_COUNT; i++)
                interp[i].target = 0.0f;
            out->lean_bias_rad  = 0.0f;
            out->step_velocity  = 0.0f;
            out->balance_active = false;
            break;
    }
}

void Gait_SetMode(RobotMode m) {
    if (m == robot_mode) return;

    // Use transition states for smooth mode changes
    if (m == MODE_WALK)  robot_mode = MODE_TRANSITION_TO_WALK;
    else if (m == MODE_DRIVE) robot_mode = MODE_TRANSITION_TO_DRIVE;
    else robot_mode = m;

    state_timer = 0.0f;
}

RobotMode   Gait_GetMode(void)       { return robot_mode; }
GaitState   Gait_GetState(void)      { return gait_state; }
void        Gait_SetWalkSpeed(float s){ walk_speed  = fmaxf(0.0f, fminf(1.0f, s)); }
void        Gait_SetStepHeight(float d){ step_height = fmaxf(0.0f, fminf(60.0f, d)); }

const char* Gait_GetModeName(void) {
    switch (robot_mode) {
        case MODE_IDLE:                return "IDLE";
        case MODE_DRIVE:               return "DRIVE";
        case MODE_WALK:                return "WALK";
        case MODE_TRANSITION_TO_WALK:  return "TO_WALK";
        case MODE_TRANSITION_TO_DRIVE: return "TO_DRIVE";
        case MODE_SAFE_SIT:            return "SIT";
        default:                       return "UNKNOWN";
    }
}
// ─────────────────────────────────────────────
//  comms.cpp  –  UART packet receiver on STM32
//  Receives 9-byte packets from ESP32
//  Validates checksum, calls Comms_OnControlPacket
// ─────────────────────────────────────────────
#pragma once

void Comms_Init(void);
void Comms_ProcessIncoming(void);   // call from gait loop ISR
void Comms_OnControlPacket(float velocity, float yaw, bool walk); // implement in main.cpp
// ─────────────────────────────────────────────
//  comms.cpp  –  UART packet receiver (STM32)
// ─────────────────────────────────────────────

#include "comms.h"
#include "config.h"
#include "gait.h"
#include "balance.h"
#include "stm32f4xx_hal.h"
#include <cstring>

extern UART_HandleTypeDef huart2;

#define PKT_START   0xAA
#define PKT_END     0x55
#define PKT_SIZE    9

static uint8_t rx_dma_buf[PKT_SIZE * 2];   // double buffer
static uint8_t pkt_buf[PKT_SIZE];
static uint8_t rx_byte_comms;

void Comms_Init(void) {
    HAL_UART_Receive_IT(&huart2, &rx_byte_comms, 1);
}

// Called from UART2 RX interrupt
static uint8_t ring[PKT_SIZE * 4];
static uint16_t ring_head = 0, ring_tail = 0;

void HAL_UART2_RxCpltCallback(void) {
    ring[ring_head++ % sizeof(ring)] = rx_byte_comms;
    HAL_UART_Receive_IT(&huart2, &rx_byte_comms, 1);
}

void Comms_ProcessIncoming(void) {
    // Try to find a valid packet in ring buffer
    while (ring_tail != ring_head) {
        uint8_t b = ring[ring_tail % sizeof(ring)];

        if (b != PKT_START) {
            ring_tail++;
            continue;
        }

        // Check we have enough bytes
        uint16_t avail = ring_head - ring_tail;
        if (avail < PKT_SIZE) return;

        // Copy candidate packet
        for (int i = 0; i < PKT_SIZE; i++)
            pkt_buf[i] = ring[(ring_tail + i) % sizeof(ring)];

        // Verify end marker
        if (pkt_buf[PKT_SIZE-1] != PKT_END) {
            ring_tail++;
            continue;
        }

        // Verify XOR checksum
        uint8_t chk = 0;
        for (int i = 1; i <= 6; i++) chk ^= pkt_buf[i];
        if (chk != pkt_buf[7]) {
            ring_tail++;
            continue;
        }

        // Valid packet — decode
        int16_t vel_i = (int16_t)((pkt_buf[1] << 8) | pkt_buf[2]);
        int16_t yaw_i = (int16_t)((pkt_buf[3] << 8) | pkt_buf[4]);
        uint8_t flags = pkt_buf[5];
        uint8_t spd   = pkt_buf[6];

        float velocity = vel_i / 1000.0f;
        float yaw      = yaw_i / 1000.0f;
        bool  walk     = (flags & 0x01) != 0;
        bool  sit      = (flags & 0x02) != 0;
        bool  use_lqr  = (flags & 0x04) != 0;

        // Apply mode commands
        if (sit) {
            Gait_SetMode(MODE_SAFE_SIT);
        } else {
            Balance_SetMode(use_lqr ? BALANCE_LQR : BALANCE_PID);
            Gait_SetWalkSpeed(spd / 255.0f);
        }

        // Deliver velocity/yaw to balance controller via callback
        Comms_OnControlPacket(velocity, yaw, walk);

        ring_tail += PKT_SIZE;
        return;
    }
}
// ─────────────────────────────────────────────
//  tuning.h / tuning.cpp  –  Serial PID tuner
//
//  Connect via USB-serial (ST-Link VCP) at 115200.
//  Commands are single-line ASCII, terminated with \n
//
//  COMMAND FORMAT:
//  <target>.<param>=<value>
//
//  Targets:
//    ip  = inner PID
//    op  = outer PID
//    lqr = LQR gains
//    fil = filter
//    gait= gait params
//
//  Params for PID: kp, ki, kd, max
//  Params for LQR: k1, k2, k3, k4
//  Params for filter: alpha, type (0=comp, 1=madgwick)
//  Params for gait: speed, step
//
//  EXAMPLES:
//    ip.kp=85.0       → set inner P gain to 85
//    op.ki=0.05       → set outer I gain to 0.05
//    lqr.k1=50.0      → set LQR tilt gain
//    fil.type=1       → switch to Madgwick filter
//    gait.speed=0.7   → set walk speed to 70%
//    ?                → print all current values
//    save             → save to flash (future feature)
//    mode=walk        → switch to walk mode
//    mode=drive       → switch to drive mode
// ─────────────────────────────────────────────
#pragma once

void Tuning_Init(void);
void Tuning_Process(void);      // call every main loop iteration
void Tuning_PrintAll(void);     // print all current gains over serial
// ─────────────────────────────────────────────
//  tuning.cpp  –  Serial PID tuning interface
// ─────────────────────────────────────────────

#include "tuning.h"
#include "balance.h"
#include "filter.h"
#include "gait.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern UART_HandleTypeDef huart1;   // tuning UART (ST-Link VCP)

// ── RX ring buffer ─────────────────────────────
#define RX_BUF_SIZE 128
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_byte;         // single-byte DMA target
static uint16_t rx_head = 0;

// Called from UART RX interrupt — append received byte to buffer
void Tuning_UART_RxCallback(void) {
    if (rx_head < RX_BUF_SIZE - 1)
        rx_buf[rx_head++] = (char)rx_byte;
    // Restart single-byte receive
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// ── Printf over UART ──────────────────────────
static void uart_print(const char* str) {
    HAL_UART_Transmit(&huart1,
                      (uint8_t*)str,
                      (uint16_t)strlen(str),
                      100);
}

static char printf_buf[256];
#define UPRINTF(fmt, ...) \
    do { snprintf(printf_buf, sizeof(printf_buf), fmt, ##__VA_ARGS__); \
         uart_print(printf_buf); } while(0)

// ── Command parser ────────────────────────────
static void parse_command(char* line) {
    // Trim trailing whitespace / CR / LF
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n'
                    || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) return;

    // '?' = print all current values
    if (strcmp(line, "?") == 0) {
        Tuning_PrintAll();
        return;
    }

    // 'mode=walk' / 'mode=drive' / 'mode=idle'
    if (strncmp(line, "mode=", 5) == 0) {
        const char* m = line + 5;
        if      (strcmp(m, "walk")  == 0) Gait_SetMode(MODE_WALK);
        else if (strcmp(m, "drive") == 0) Gait_SetMode(MODE_DRIVE);
        else if (strcmp(m, "idle")  == 0) Gait_SetMode(MODE_IDLE);
        else if (strcmp(m, "sit")   == 0) Gait_SetMode(MODE_SAFE_SIT);
        UPRINTF("mode -> %s\r\n", Gait_GetModeName());
        return;
    }

    // 'balance=pid' / 'balance=lqr'
    if (strncmp(line, "balance=", 8) == 0) {
        const char* b = line + 8;
        if      (strcmp(b, "pid") == 0) Balance_SetMode(BALANCE_PID);
        else if (strcmp(b, "lqr") == 0) Balance_SetMode(BALANCE_LQR);
        uart_print("balance mode updated\r\n");
        return;
    }

    // Standard format: target.param=value
    char* dot   = strchr(line, '.');
    char* eq    = strchr(line, '=');
    if (!dot || !eq || dot > eq) {
        uart_print("ERR: bad format. Use target.param=value\r\n");
        return;
    }

    *dot = '\0'; *eq = '\0';
    const char* target = line;
    const char* param  = dot + 1;
    float       value  = strtof(eq + 1, nullptr);

    PIDGains inner, outer;
    Balance_GetGains(&inner, &outer);

    if (strcmp(target, "ip") == 0) {
        // Inner PID
        if      (strcmp(param, "kp")  == 0) inner.kp         = value;
        else if (strcmp(param, "ki")  == 0) inner.ki         = value;
        else if (strcmp(param, "kd")  == 0) inner.kd         = value;
        else if (strcmp(param, "max") == 0) inner.max_output = value;
        else { uart_print("ERR: unknown param\r\n"); return; }
        Balance_SetGains(&inner, nullptr);

    } else if (strcmp(target, "op") == 0) {
        // Outer PID
        if      (strcmp(param, "kp")  == 0) outer.kp         = value;
        else if (strcmp(param, "ki")  == 0) outer.ki         = value;
        else if (strcmp(param, "kd")  == 0) outer.kd         = value;
        else if (strcmp(param, "max") == 0) outer.max_output = value;
        else { uart_print("ERR: unknown param\r\n"); return; }
        Balance_SetGains(nullptr, &outer);

    } else if (strcmp(target, "fil") == 0) {
        if (strcmp(param, "type") == 0)
            Filter_SetType((int)value == 0 ? FILTER_COMPLEMENTARY
                                           : FILTER_MADGWICK);
        else { uart_print("ERR: unknown param\r\n"); return; }

    } else if (strcmp(target, "gait") == 0) {
        if      (strcmp(param, "speed") == 0) Gait_SetWalkSpeed(value);
        else if (strcmp(param, "step")  == 0) Gait_SetStepHeight(value);
        else { uart_print("ERR: unknown param\r\n"); return; }

    } else {
        uart_print("ERR: unknown target\r\n");
        return;
    }

    UPRINTF("OK: %s.%s = %.4f\r\n", target, param, value);
}

// ── Public API ────────────────────────────────
void Tuning_Init(void) {
    // Start interrupt-driven receive
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    uart_print("\r\n=== Robot Tuning Interface ===\r\n");
    uart_print("Type '?' for current values\r\n");
    uart_print("Format: target.param=value\r\n");
    uart_print("Targets: ip op fil gait\r\n");
    uart_print("Modes: mode=walk/drive/idle/sit\r\n\r\n");
}

void Tuning_Process(void) {
    // Check if a newline has arrived in the buffer
    for (uint16_t i = 0; i < rx_head; i++) {
        if (rx_buf[i] == '\n') {
            rx_buf[i] = '\0';
            parse_command(rx_buf);
            // Shift remaining bytes down
            uint16_t remaining = rx_head - i - 1;
            memmove(rx_buf, rx_buf + i + 1, remaining);
            rx_head = remaining;
            return;
        }
    }
}

void Tuning_PrintAll(void) {
    PIDGains inner, outer;
    Balance_GetGains(&inner, &outer);

    uart_print("\r\n--- Current Settings ---\r\n");
    UPRINTF("Inner PID:  kp=%.3f  ki=%.3f  kd=%.3f  max=%.1f\r\n",
            inner.kp, inner.ki, inner.kd, inner.max_output);
    UPRINTF("Outer PID:  kp=%.3f  ki=%.3f  kd=%.3f  max=%.3f\r\n",
            outer.kp, outer.ki, outer.kd, outer.max_output);
    UPRINTF("LQR Gains:  k1=%.3f  k2=%.3f  k3=%.3f  k4=%.3f\r\n",
            LQR_K1, LQR_K2, LQR_K3, LQR_K4);
    UPRINTF("Mode: %s\r\n", Gait_GetModeName());
    uart_print("------------------------\r\n\r\n");
}
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
// ─────────────────────────────────────────────
//  ESP32 main.cpp  –  Game controller + comms
//
//  Runs on ESP32-S3 (or standard ESP32)
//  Libraries required:
//    - Bluepad32 (install via Arduino Library Manager)
//    - Arduino ESP32 core
//
//  FUNCTION:
//  1. Pairs with PS4/PS5/Xbox/Nintendo controller
//     via Bluetooth using Bluepad32
//  2. Reads joystick axes and buttons
//  3. Packs into a binary control packet
//  4. Sends to STM32 over UART at COMMS_HZ rate
//
//  PACKET FORMAT (9 bytes, binary):
//  Byte 0:   0xAA (start marker)
//  Byte 1-2: velocity (int16, mm/s, ×1000 of m/s)
//  Byte 3-4: yaw rate (int16, mrad/s)
//  Byte 5:   flags (bit0=walk_mode, bit1=sit, bit2=pid/lqr)
//  Byte 6:   walk speed (uint8, 0–255)
//  Byte 7:   checksum (XOR of bytes 1–6)
//  Byte 8:   0x55 (end marker)
//
//  CONTROLLER MAPPING (PS4/PS5):
//  Left stick Y  → forward/backward velocity
//  Right stick X → yaw (turn)
//  L1 button     → hold = walk mode, release = drive mode
//  R1 button     → increase walk speed
//  Cross (X)     → emergency sit
//  Triangle      → toggle PID/LQR
//  Options       → recalibrate IMU (sends command to STM32)
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <Bluepad32.h>

// ── UART to STM32 ─────────────────────────────
// ESP32-S3: Serial1 on GPIO17 (TX) and GPIO18 (RX)
#define STM32_UART      Serial1
#define STM32_BAUD      460800
#define STM32_TX_PIN    17
#define STM32_RX_PIN    18

// ── Packet constants ──────────────────────────
#define PKT_START       0xAA
#define PKT_END         0x55
#define PKT_SIZE        9
#define COMMS_HZ        100
#define COMMS_PERIOD_MS (1000 / COMMS_HZ)

// ── Controller state ──────────────────────────
static ControllerPtr active_controller = nullptr;

// Smoothed command values
static float smooth_velocity = 0.0f;   // m/s
static float smooth_yaw      = 0.0f;   // rad/s
static bool  walk_mode        = false;
static bool  lqr_mode         = false;
static float walk_speed       = 0.5f;  // 0–1

// Deadzone for joystick axes (0–512 range from Bluepad32)
#define AXIS_MAX        512
#define DEADZONE        30
#define MAX_VELOCITY    0.5f    // m/s
#define MAX_YAW_RATE    2.0f    // rad/s

// Low-pass filter coefficient for stick smoothing
#define SMOOTH_ALPHA    0.2f

// ── Bluepad32 callbacks ───────────────────────
void onConnectedController(ControllerPtr ctl) {
    active_controller = ctl;
    Serial.println("Controller connected");
    // Rumble briefly to confirm connection
    ctl->setRumble(0x80, 0x40);
    delay(200);
    ctl->setRumble(0, 0);
}

void onDisconnectedController(ControllerPtr ctl) {
    if (active_controller == ctl) {
        active_controller = nullptr;
        Serial.println("Controller disconnected");
    }
}

// ── Joystick axis to float ────────────────────
// Bluepad32 returns axis values −512 to +512
// We apply deadzone and map to −1.0 to +1.0
static float axis_to_float(int32_t raw) {
    if (abs(raw) < DEADZONE) return 0.0f;
    float normalised = (float)(raw - (raw > 0 ? DEADZONE : -DEADZONE))
                     / (float)(AXIS_MAX - DEADZONE);
    return constrain(normalised, -1.0f, 1.0f);
}

// ── Build and send packet ─────────────────────
static uint8_t packet[PKT_SIZE];

static void send_packet(float velocity, float yaw,
                        bool walk, bool sit,
                        bool use_lqr, uint8_t spd) {
    // Scale to integer representation
    int16_t vel_i = (int16_t)(velocity  * 1000.0f);  // mm/s
    int16_t yaw_i = (int16_t)(yaw       * 1000.0f);  // mrad/s

    uint8_t flags = (walk    ? 0x01 : 0)
                  | (sit     ? 0x02 : 0)
                  | (use_lqr ? 0x04 : 0);

    packet[0] = PKT_START;
    packet[1] = (uint8_t)(vel_i >> 8);
    packet[2] = (uint8_t)(vel_i & 0xFF);
    packet[3] = (uint8_t)(yaw_i >> 8);
    packet[4] = (uint8_t)(yaw_i & 0xFF);
    packet[5] = flags;
    packet[6] = spd;

    // XOR checksum over payload bytes
    uint8_t chk = 0;
    for (int i = 1; i <= 6; i++) chk ^= packet[i];
    packet[7] = chk;
    packet[8] = PKT_END;

    STM32_UART.write(packet, PKT_SIZE);
}

// ── Button edge detection ─────────────────────
static uint16_t prev_buttons = 0;

static bool button_just_pressed(uint16_t buttons, uint16_t mask) {
    return (buttons & mask) && !(prev_buttons & mask);
}

// ── Setup ─────────────────────────────────────
void setup() {
    Serial.begin(115200);   // USB debug
    STM32_UART.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

    // Initialise Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Forget previously paired controllers to force fresh pairing
    // Comment this out after first successful pairing
    BP32.forgetBluetoothKeys();

    Serial.println("Bluepad32 ready. Pair your controller now.");
}

// ── Main loop ─────────────────────────────────
void loop() {
    static uint32_t last_send_ms = 0;
    bool sit_pressed = false;

    // Process Bluepad32 (must be called every loop)
    BP32.update();

    if (active_controller && active_controller->isConnected()) {
        uint16_t buttons = active_controller->buttons();

        // ── Velocity: left stick Y axis ───────
        // Bluepad32 Y axis: negative = up (forward)
        float vel_raw = -axis_to_float(active_controller->axisY())
                      * MAX_VELOCITY;

        // ── Yaw: right stick X axis ───────────
        float yaw_raw = axis_to_float(active_controller->axisRX())
                      * MAX_YAW_RATE;

        // Low-pass smooth to reduce jitter
        smooth_velocity = SMOOTH_ALPHA * vel_raw
                        + (1.0f - SMOOTH_ALPHA) * smooth_velocity;
        smooth_yaw      = SMOOTH_ALPHA * yaw_raw
                        + (1.0f - SMOOTH_ALPHA) * smooth_yaw;

        // ── Walk mode: L1 button ──────────────
        // BUTTON_L1 = 0x0010 in Bluepad32
        walk_mode = (buttons & BUTTON_L1) != 0;

        // ── Walk speed: R1 button ─────────────
        if (button_just_pressed(buttons, BUTTON_R1)) {
            walk_speed += 0.2f;
            if (walk_speed > 1.0f) walk_speed = 0.2f;
        }

        // ── Emergency sit: Cross button ───────
        sit_pressed = button_just_pressed(buttons, BUTTON_CROSS);

        // ── Toggle PID/LQR: Triangle ──────────
        if (button_just_pressed(buttons, BUTTON_TRIANGLE))
            lqr_mode = !lqr_mode;

        // Debug output
        Serial.printf("vel=%.2f yaw=%.2f walk=%d lqr=%d spd=%.1f\n",
                      smooth_velocity, smooth_yaw,
                      walk_mode, lqr_mode, walk_speed);

        prev_buttons = buttons;
    } else {
        // No controller — zero commands (safe state)
        smooth_velocity = 0.0f;
        smooth_yaw      = 0.0f;
    }

    // Send at fixed rate regardless of controller state
    uint32_t now = millis();
    if (now - last_send_ms >= COMMS_PERIOD_MS) {
        send_packet(smooth_velocity, smooth_yaw,
                    walk_mode, sit_pressed, lqr_mode,
                    (uint8_t)(walk_speed * 255.0f));
        last_send_ms = now;
    }
}
