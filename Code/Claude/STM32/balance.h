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
