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
