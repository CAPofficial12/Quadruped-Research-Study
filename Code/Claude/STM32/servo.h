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
