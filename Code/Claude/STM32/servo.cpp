// ─────────────────────────────────────────────
//  servo.cpp  –  Servo PWM via TIM3
//
//  TIM3 is configured for 50Hz PWM (20ms period)
//  with 1µs resolution (1MHz timer clock).
//
//  Timer setup:
//    APB1 clock = 84MHz (STM32F405 default)
//    Prescaler  = 83  → timer clock = 1MHz
//    ARR        = 19999 → period = 20ms = 50Hz
//
//  CCR register value directly = pulse width in µs
//  CCR = 1000 → 1ms  = full reverse
//  CCR = 1500 → 1.5ms = neutral
//  CCR = 2000 → 2ms  = full forward
//
//  All 4 channels share the same timer so all
//  servo pulses start at the same time each
//  cycle — no sequential jitter.
// ─────────────────────────────────────────────

#include "servo.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <algorithm>
#include <cmath>

// ── TIM3 direct register pointers ─────────────
// Using direct register access is faster and
// more explicit than the HAL for runtime updates.
// TIM3 base address from STM32F4 reference manual
#define TIM3_BASE_ADDR   0x40000400UL
#define TIM3_CR1    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x00))
#define TIM3_ARR    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x2C))
#define TIM3_PSC    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x28))
#define TIM3_CCR1   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x34))
#define TIM3_CCR2   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x38))
#define TIM3_CCR3   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x3C))
#define TIM3_CCR4   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x40))
#define TIM3_CCMR1  (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x18))
#define TIM3_CCMR2  (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x1C))
#define TIM3_CCER   (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x20))
#define TIM3_EGR    (*(volatile uint32_t*)(TIM3_BASE_ADDR + 0x14))
#define RCC_APB1ENR (*(volatile uint32_t*)(0x40023840UL))

// ── Internal state ────────────────────────────
static uint16_t pulse_us[SERVO_COUNT];
static float    angle_deg[SERVO_COUNT];

// Pointer array: index → CCR register for fast write
static volatile uint32_t* const ccr[SERVO_COUNT] = {
    &TIM3_CCR1,  // SERVO_HIP_LEFT
    &TIM3_CCR2,  // SERVO_HIP_RIGHT
    &TIM3_CCR3,  // SERVO_KNEE_LEFT
    &TIM3_CCR4   // SERVO_KNEE_RIGHT
};

// ── Angle to pulse width conversion ───────────
// Maps degrees (−90 to +90) to pulse width (1000–2000µs)
// Neutral (0°) = 1500µs
static uint16_t angle_to_us(float deg) {
    // Clamp to servo mechanical limits
    float clamped = fmaxf(-90.0f, fminf(90.0f, deg));
    // Linear map: 0° = 1500µs, ±90° = ±500µs
    float us = SERVO_PWM_MID_US + (clamped / 90.0f) * 500.0f;
    return (uint16_t)fmaxf(SERVO_PWM_MIN_US,
                   fminf(SERVO_PWM_MAX_US, us));
}

// ── Initialise TIM3 for 50Hz PWM ─────────────
void Servo_Init(void) {
    // Enable TIM3 clock via RCC APB1ENR bit 1
    RCC_APB1ENR |= (1 << 1);

    // GPIO setup: PA6, PA7, PB0, PB1 as AF2 (TIM3)
    // This is done in CubeMX — verify your pin config matches config.h

    // Timer configuration
    TIM3_CR1  = 0;           // disable timer during config
    TIM3_PSC  = 83;          // 84MHz / (83+1) = 1MHz timer clock
    TIM3_ARR  = 19999;       // 1MHz / 20000 = 50Hz

    // PWM mode 1 on all 4 channels:
    // CCMR1: CH1 and CH2, CCMR2: CH3 and CH4
    // OC1M/OC2M/OC3M/OC4M = 110 (PWM mode 1)
    // OCxPE = 1 (preload enable — updates take effect at next update event)
    TIM3_CCMR1 = (0x68 << 0)  // CH1: PWM mode 1, preload
               | (0x68 << 8); // CH2: PWM mode 1, preload
    TIM3_CCMR2 = (0x68 << 0)  // CH3: PWM mode 1, preload
               | (0x68 << 8); // CH4: PWM mode 1, preload

    // Enable all 4 capture/compare outputs
    // CC1E, CC2E, CC3E, CC4E bits in CCER
    TIM3_CCER = (1<<0) | (1<<4) | (1<<8) | (1<<12);

    // Set all servos to neutral (1500µs)
    for (int i = 0; i < SERVO_COUNT; i++) {
        pulse_us[i]  = SERVO_PWM_MID_US;
        angle_deg[i] = 0.0f;
        *ccr[i] = SERVO_PWM_MID_US;
    }

    // Generate update event to load preload registers
    TIM3_EGR = 1;

    // Enable timer
    TIM3_CR1 = 1;
}

// ── Public API ────────────────────────────────
void Servo_SetAngle(ServoID id, float degrees) {
    if (id >= SERVO_COUNT) return;

    // Apply mechanical limits from config
    float limited = degrees;
    if (id == SERVO_HIP_LEFT || id == SERVO_HIP_RIGHT) {
        limited = fmaxf(HIP_MAX_BACKWARD_DEG,
                  fminf(HIP_MAX_FORWARD_DEG, degrees));
    } else {
        limited = fmaxf(KNEE_MAX_EXTEND_DEG,
                  fminf(KNEE_MAX_FLEX_DEG, degrees));
    }

    angle_deg[id] = limited;
    pulse_us[id]  = angle_to_us(limited);
    *ccr[id]      = pulse_us[id];   // direct CCR write — takes effect next cycle
}

void Servo_SetPulse(ServoID id, uint16_t us) {
    if (id >= SERVO_COUNT) return;
    uint16_t clamped = (uint16_t)fmaxf(SERVO_PWM_MIN_US,
                                fminf(SERVO_PWM_MAX_US, us));
    pulse_us[id]  = clamped;
    angle_deg[id] = ((float)clamped - SERVO_PWM_MID_US) / 500.0f * 90.0f;
    *ccr[id]      = clamped;
}

void Servo_SetAll(float hip_l, float hip_r, float knee_l, float knee_r) {
    // Write all 4 CCR registers back-to-back for minimum skew
    // All updates take effect at the next PWM cycle start
    TIM3_CCR1 = angle_to_us(fmaxf(HIP_MAX_BACKWARD_DEG,
                             fminf(HIP_MAX_FORWARD_DEG, hip_l)));
    TIM3_CCR2 = angle_to_us(fmaxf(HIP_MAX_BACKWARD_DEG,
                             fminf(HIP_MAX_FORWARD_DEG, hip_r)));
    TIM3_CCR3 = angle_to_us(fmaxf(KNEE_MAX_EXTEND_DEG,
                             fminf(KNEE_MAX_FLEX_DEG, knee_l)));
    TIM3_CCR4 = angle_to_us(fmaxf(KNEE_MAX_EXTEND_DEG,
                             fminf(KNEE_MAX_FLEX_DEG, knee_r)));

    angle_deg[0] = hip_l;  angle_deg[1] = hip_r;
    angle_deg[2] = knee_l; angle_deg[3] = knee_r;
}

void Servo_Park(void) {
    Servo_SetAll(0.0f, 0.0f, 0.0f, 0.0f);
}

void Servo_Disable(void) {
    // Clear CC output enables — servos go slack (no holding torque)
    TIM3_CCER = 0;
}

float Servo_GetAngle(ServoID id) {
    if (id >= SERVO_COUNT) return 0.0f;
    return angle_deg[id];
}
