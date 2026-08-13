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
