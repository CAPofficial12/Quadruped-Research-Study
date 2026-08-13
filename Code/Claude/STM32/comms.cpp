// ─────────────────────────────────────────────
//  comms.cpp  –  UART packet receiver (STM32)
// ─────────────────────────────────────────────

#include "comms.h"
#include "config.h"
#include "gait.h"
#include "balance.h"
#include "stm32f4xx_hal.h"
#include <cstring>

extern UART_HandleTypeDef huart2;

#define PKT_START   0xAA
#define PKT_END     0x55
#define PKT_SIZE    9

static uint8_t rx_dma_buf[PKT_SIZE * 2];   // double buffer
static uint8_t pkt_buf[PKT_SIZE];
static uint8_t rx_byte_comms;

void Comms_Init(void) {
    HAL_UART_Receive_IT(&huart2, &rx_byte_comms, 1);
}

// Called from UART2 RX interrupt
static uint8_t ring[PKT_SIZE * 4];
static uint16_t ring_head = 0, ring_tail = 0;

void HAL_UART2_RxCpltCallback(void) {
    ring[ring_head++ % sizeof(ring)] = rx_byte_comms;
    HAL_UART_Receive_IT(&huart2, &rx_byte_comms, 1);
}

void Comms_ProcessIncoming(void) {
    // Try to find a valid packet in ring buffer
    while (ring_tail != ring_head) {
        uint8_t b = ring[ring_tail % sizeof(ring)];

        if (b != PKT_START) {
            ring_tail++;
            continue;
        }

        // Check we have enough bytes
        uint16_t avail = ring_head - ring_tail;
        if (avail < PKT_SIZE) return;

        // Copy candidate packet
        for (int i = 0; i < PKT_SIZE; i++)
            pkt_buf[i] = ring[(ring_tail + i) % sizeof(ring)];

        // Verify end marker
        if (pkt_buf[PKT_SIZE-1] != PKT_END) {
            ring_tail++;
            continue;
        }

        // Verify XOR checksum
        uint8_t chk = 0;
        for (int i = 1; i <= 6; i++) chk ^= pkt_buf[i];
        if (chk != pkt_buf[7]) {
            ring_tail++;
            continue;
        }

        // Valid packet — decode
        int16_t vel_i = (int16_t)((pkt_buf[1] << 8) | pkt_buf[2]);
        int16_t yaw_i = (int16_t)((pkt_buf[3] << 8) | pkt_buf[4]);
        uint8_t flags = pkt_buf[5];
        uint8_t spd   = pkt_buf[6];

        float velocity = vel_i / 1000.0f;
        float yaw      = yaw_i / 1000.0f;
        bool  walk     = (flags & 0x01) != 0;
        bool  sit      = (flags & 0x02) != 0;
        bool  use_lqr  = (flags & 0x04) != 0;

        // Apply mode commands
        if (sit) {
            Gait_SetMode(MODE_SAFE_SIT);
        } else {
            Balance_SetMode(use_lqr ? BALANCE_LQR : BALANCE_PID);
            Gait_SetWalkSpeed(spd / 255.0f);
        }

        // Deliver velocity/yaw to balance controller via callback
        Comms_OnControlPacket(velocity, yaw, walk);

        ring_tail += PKT_SIZE;
        return;
    }
}
