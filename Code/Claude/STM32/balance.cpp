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
