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
