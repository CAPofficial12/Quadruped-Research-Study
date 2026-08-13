// ─────────────────────────────────────────────
//  tuning.cpp  –  Serial PID tuning interface
// ─────────────────────────────────────────────

#include "tuning.h"
#include "balance.h"
#include "filter.h"
#include "gait.h"
#include "config.h"
#include "stm32f4xx_hal.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern UART_HandleTypeDef huart1;   // tuning UART (ST-Link VCP)

// ── RX ring buffer ─────────────────────────────
#define RX_BUF_SIZE 128
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_byte;         // single-byte DMA target
static uint16_t rx_head = 0;

// Called from UART RX interrupt — append received byte to buffer
void Tuning_UART_RxCallback(void) {
    if (rx_head < RX_BUF_SIZE - 1)
        rx_buf[rx_head++] = (char)rx_byte;
    // Restart single-byte receive
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// ── Printf over UART ──────────────────────────
static void uart_print(const char* str) {
    HAL_UART_Transmit(&huart1,
                      (uint8_t*)str,
                      (uint16_t)strlen(str),
                      100);
}

static char printf_buf[256];
#define UPRINTF(fmt, ...) \
    do { snprintf(printf_buf, sizeof(printf_buf), fmt, ##__VA_ARGS__); \
         uart_print(printf_buf); } while(0)

// ── Command parser ────────────────────────────
static void parse_command(char* line) {
    // Trim trailing whitespace / CR / LF
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n'
                    || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) return;

    // '?' = print all current values
    if (strcmp(line, "?") == 0) {
        Tuning_PrintAll();
        return;
    }

    // 'mode=walk' / 'mode=drive' / 'mode=idle'
    if (strncmp(line, "mode=", 5) == 0) {
        const char* m = line + 5;
        if      (strcmp(m, "walk")  == 0) Gait_SetMode(MODE_WALK);
        else if (strcmp(m, "drive") == 0) Gait_SetMode(MODE_DRIVE);
        else if (strcmp(m, "idle")  == 0) Gait_SetMode(MODE_IDLE);
        else if (strcmp(m, "sit")   == 0) Gait_SetMode(MODE_SAFE_SIT);
        UPRINTF("mode -> %s\r\n", Gait_GetModeName());
        return;
    }

    // 'balance=pid' / 'balance=lqr'
    if (strncmp(line, "balance=", 8) == 0) {
        const char* b = line + 8;
        if      (strcmp(b, "pid") == 0) Balance_SetMode(BALANCE_PID);
        else if (strcmp(b, "lqr") == 0) Balance_SetMode(BALANCE_LQR);
        uart_print("balance mode updated\r\n");
        return;
    }

    // Standard format: target.param=value
    char* dot   = strchr(line, '.');
    char* eq    = strchr(line, '=');
    if (!dot || !eq || dot > eq) {
        uart_print("ERR: bad format. Use target.param=value\r\n");
        return;
    }

    *dot = '\0'; *eq = '\0';
    const char* target = line;
    const char* param  = dot + 1;
    float       value  = strtof(eq + 1, nullptr);

    PIDGains inner, outer;
    Balance_GetGains(&inner, &outer);

    if (strcmp(target, "ip") == 0) {
        // Inner PID
        if      (strcmp(param, "kp")  == 0) inner.kp         = value;
        else if (strcmp(param, "ki")  == 0) inner.ki         = value;
        else if (strcmp(param, "kd")  == 0) inner.kd         = value;
        else if (strcmp(param, "max") == 0) inner.max_output = value;
        else { uart_print("ERR: unknown param\r\n"); return; }
        Balance_SetGains(&inner, nullptr);

    } else if (strcmp(target, "op") == 0) {
        // Outer PID
        if      (strcmp(param, "kp")  == 0) outer.kp         = value;
        else if (strcmp(param, "ki")  == 0) outer.ki         = value;
        else if (strcmp(param, "kd")  == 0) outer.kd         = value;
        else if (strcmp(param, "max") == 0) outer.max_output = value;
        else { uart_print("ERR: unknown param\r\n"); return; }
        Balance_SetGains(nullptr, &outer);

    } else if (strcmp(target, "fil") == 0) {
        if (strcmp(param, "type") == 0)
            Filter_SetType((int)value == 0 ? FILTER_COMPLEMENTARY
                                           : FILTER_MADGWICK);
        else { uart_print("ERR: unknown param\r\n"); return; }

    } else if (strcmp(target, "gait") == 0) {
        if      (strcmp(param, "speed") == 0) Gait_SetWalkSpeed(value);
        else if (strcmp(param, "step")  == 0) Gait_SetStepHeight(value);
        else { uart_print("ERR: unknown param\r\n"); return; }

    } else {
        uart_print("ERR: unknown target\r\n");
        return;
    }

    UPRINTF("OK: %s.%s = %.4f\r\n", target, param, value);
}

// ── Public API ────────────────────────────────
void Tuning_Init(void) {
    // Start interrupt-driven receive
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    uart_print("\r\n=== Robot Tuning Interface ===\r\n");
    uart_print("Type '?' for current values\r\n");
    uart_print("Format: target.param=value\r\n");
    uart_print("Targets: ip op fil gait\r\n");
    uart_print("Modes: mode=walk/drive/idle/sit\r\n\r\n");
}

void Tuning_Process(void) {
    // Check if a newline has arrived in the buffer
    for (uint16_t i = 0; i < rx_head; i++) {
        if (rx_buf[i] == '\n') {
            rx_buf[i] = '\0';
            parse_command(rx_buf);
            // Shift remaining bytes down
            uint16_t remaining = rx_head - i - 1;
            memmove(rx_buf, rx_buf + i + 1, remaining);
            rx_head = remaining;
            return;
        }
    }
}

void Tuning_PrintAll(void) {
    PIDGains inner, outer;
    Balance_GetGains(&inner, &outer);

    uart_print("\r\n--- Current Settings ---\r\n");
    UPRINTF("Inner PID:  kp=%.3f  ki=%.3f  kd=%.3f  max=%.1f\r\n",
            inner.kp, inner.ki, inner.kd, inner.max_output);
    UPRINTF("Outer PID:  kp=%.3f  ki=%.3f  kd=%.3f  max=%.3f\r\n",
            outer.kp, outer.ki, outer.kd, outer.max_output);
    UPRINTF("LQR Gains:  k1=%.3f  k2=%.3f  k3=%.3f  k4=%.3f\r\n",
            LQR_K1, LQR_K2, LQR_K3, LQR_K4);
    UPRINTF("Mode: %s\r\n", Gait_GetModeName());
    uart_print("------------------------\r\n\r\n");
}
