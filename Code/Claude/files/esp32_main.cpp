// ─────────────────────────────────────────────
//  ESP32 main.cpp  –  Game controller + comms
//
//  Runs on ESP32-S3 (or standard ESP32)
//  Libraries required:
//    - Bluepad32 (install via Arduino Library Manager)
//    - Arduino ESP32 core
//
//  FUNCTION:
//  1. Pairs with PS4/PS5/Xbox/Nintendo controller
//     via Bluetooth using Bluepad32
//  2. Reads joystick axes and buttons
//  3. Packs into a binary control packet
//  4. Sends to STM32 over UART at COMMS_HZ rate
//
//  PACKET FORMAT (9 bytes, binary):
//  Byte 0:   0xAA (start marker)
//  Byte 1-2: velocity (int16, mm/s, ×1000 of m/s)
//  Byte 3-4: yaw rate (int16, mrad/s)
//  Byte 5:   flags (bit0=walk_mode, bit1=sit, bit2=pid/lqr)
//  Byte 6:   walk speed (uint8, 0–255)
//  Byte 7:   checksum (XOR of bytes 1–6)
//  Byte 8:   0x55 (end marker)
//
//  CONTROLLER MAPPING (PS4/PS5):
//  Left stick Y  → forward/backward velocity
//  Right stick X → yaw (turn)
//  L1 button     → hold = walk mode, release = drive mode
//  R1 button     → increase walk speed
//  Cross (X)     → emergency sit
//  Triangle      → toggle PID/LQR
//  Options       → recalibrate IMU (sends command to STM32)
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <Bluepad32.h>

// ── UART to STM32 ─────────────────────────────
// ESP32-S3: Serial1 on GPIO17 (TX) and GPIO18 (RX)
#define STM32_UART      Serial1
#define STM32_BAUD      460800
#define STM32_TX_PIN    17
#define STM32_RX_PIN    18

// ── Packet constants ──────────────────────────
#define PKT_START       0xAA
#define PKT_END         0x55
#define PKT_SIZE        9
#define COMMS_HZ        100
#define COMMS_PERIOD_MS (1000 / COMMS_HZ)

// ── Controller state ──────────────────────────
static ControllerPtr active_controller = nullptr;

// Smoothed command values
static float smooth_velocity = 0.0f;   // m/s
static float smooth_yaw      = 0.0f;   // rad/s
static bool  walk_mode        = false;
static bool  lqr_mode         = false;
static float walk_speed       = 0.5f;  // 0–1

// Deadzone for joystick axes (0–512 range from Bluepad32)
#define AXIS_MAX        512
#define DEADZONE        30
#define MAX_VELOCITY    0.5f    // m/s
#define MAX_YAW_RATE    2.0f    // rad/s

// Low-pass filter coefficient for stick smoothing
#define SMOOTH_ALPHA    0.2f

// ── Bluepad32 callbacks ───────────────────────
void onConnectedController(ControllerPtr ctl) {
    active_controller = ctl;
    Serial.println("Controller connected");
    // Rumble briefly to confirm connection
    ctl->setRumble(0x80, 0x40);
    delay(200);
    ctl->setRumble(0, 0);
}

void onDisconnectedController(ControllerPtr ctl) {
    if (active_controller == ctl) {
        active_controller = nullptr;
        Serial.println("Controller disconnected");
    }
}

// ── Joystick axis to float ────────────────────
// Bluepad32 returns axis values −512 to +512
// We apply deadzone and map to −1.0 to +1.0
static float axis_to_float(int32_t raw) {
    if (abs(raw) < DEADZONE) return 0.0f;
    float normalised = (float)(raw - (raw > 0 ? DEADZONE : -DEADZONE))
                     / (float)(AXIS_MAX - DEADZONE);
    return constrain(normalised, -1.0f, 1.0f);
}

// ── Build and send packet ─────────────────────
static uint8_t packet[PKT_SIZE];

static void send_packet(float velocity, float yaw,
                        bool walk, bool sit,
                        bool use_lqr, uint8_t spd) {
    // Scale to integer representation
    int16_t vel_i = (int16_t)(velocity  * 1000.0f);  // mm/s
    int16_t yaw_i = (int16_t)(yaw       * 1000.0f);  // mrad/s

    uint8_t flags = (walk    ? 0x01 : 0)
                  | (sit     ? 0x02 : 0)
                  | (use_lqr ? 0x04 : 0);

    packet[0] = PKT_START;
    packet[1] = (uint8_t)(vel_i >> 8);
    packet[2] = (uint8_t)(vel_i & 0xFF);
    packet[3] = (uint8_t)(yaw_i >> 8);
    packet[4] = (uint8_t)(yaw_i & 0xFF);
    packet[5] = flags;
    packet[6] = spd;

    // XOR checksum over payload bytes
    uint8_t chk = 0;
    for (int i = 1; i <= 6; i++) chk ^= packet[i];
    packet[7] = chk;
    packet[8] = PKT_END;

    STM32_UART.write(packet, PKT_SIZE);
}

// ── Button edge detection ─────────────────────
static uint16_t prev_buttons = 0;

static bool button_just_pressed(uint16_t buttons, uint16_t mask) {
    return (buttons & mask) && !(prev_buttons & mask);
}

// ── Setup ─────────────────────────────────────
void setup() {
    Serial.begin(115200);   // USB debug
    STM32_UART.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

    // Initialise Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Forget previously paired controllers to force fresh pairing
    // Comment this out after first successful pairing
    BP32.forgetBluetoothKeys();

    Serial.println("Bluepad32 ready. Pair your controller now.");
}

// ── Main loop ─────────────────────────────────
void loop() {
    static uint32_t last_send_ms = 0;
    bool sit_pressed = false;

    // Process Bluepad32 (must be called every loop)
    BP32.update();

    if (active_controller && active_controller->isConnected()) {
        uint16_t buttons = active_controller->buttons();

        // ── Velocity: left stick Y axis ───────
        // Bluepad32 Y axis: negative = up (forward)
        float vel_raw = -axis_to_float(active_controller->axisY())
                      * MAX_VELOCITY;

        // ── Yaw: right stick X axis ───────────
        float yaw_raw = axis_to_float(active_controller->axisRX())
                      * MAX_YAW_RATE;

        // Low-pass smooth to reduce jitter
        smooth_velocity = SMOOTH_ALPHA * vel_raw
                        + (1.0f - SMOOTH_ALPHA) * smooth_velocity;
        smooth_yaw      = SMOOTH_ALPHA * yaw_raw
                        + (1.0f - SMOOTH_ALPHA) * smooth_yaw;

        // ── Walk mode: L1 button ──────────────
        // BUTTON_L1 = 0x0010 in Bluepad32
        walk_mode = (buttons & BUTTON_L1) != 0;

        // ── Walk speed: R1 button ─────────────
        if (button_just_pressed(buttons, BUTTON_R1)) {
            walk_speed += 0.2f;
            if (walk_speed > 1.0f) walk_speed = 0.2f;
        }

        // ── Emergency sit: Cross button ───────
        sit_pressed = button_just_pressed(buttons, BUTTON_CROSS);

        // ── Toggle PID/LQR: Triangle ──────────
        if (button_just_pressed(buttons, BUTTON_TRIANGLE))
            lqr_mode = !lqr_mode;

        // Debug output
        Serial.printf("vel=%.2f yaw=%.2f walk=%d lqr=%d spd=%.1f\n",
                      smooth_velocity, smooth_yaw,
                      walk_mode, lqr_mode, walk_speed);

        prev_buttons = buttons;
    } else {
        // No controller — zero commands (safe state)
        smooth_velocity = 0.0f;
        smooth_yaw      = 0.0f;
    }

    // Send at fixed rate regardless of controller state
    uint32_t now = millis();
    if (now - last_send_ms >= COMMS_PERIOD_MS) {
        send_packet(smooth_velocity, smooth_yaw,
                    walk_mode, sit_pressed, lqr_mode,
                    (uint8_t)(walk_speed * 255.0f));
        last_send_ms = now;
    }
}
