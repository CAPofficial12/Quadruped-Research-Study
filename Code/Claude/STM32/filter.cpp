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
