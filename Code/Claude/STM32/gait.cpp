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
