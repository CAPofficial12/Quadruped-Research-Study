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
