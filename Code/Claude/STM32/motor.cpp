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
