// ─────────────────────────────────────────────
//  comms.cpp  –  UART packet receiver on STM32
//  Receives 9-byte packets from ESP32
//  Validates checksum, calls Comms_OnControlPacket
// ─────────────────────────────────────────────
#pragma once

void Comms_Init(void);
void Comms_ProcessIncoming(void);   // call from gait loop ISR
void Comms_OnControlPacket(float velocity, float yaw, bool walk); // implement in main.cpp
