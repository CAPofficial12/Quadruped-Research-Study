/*
  CPG Gait Controller -- Teensy 4.1
  "Parameter Sensitivity of CPG Locomotion Controllers in a Quadruped Robot"

  Implements a coupled Hopf-oscillator CPG (phase + amplitude per leg),
  three gaits (Walk / Trot / Pace) via phase-bias matrices, RK4 integration,
  and a serial-driven trial state machine for running the 540-trial study.

  ---------------------------------------------------------------------------
  ASSUMPTIONS TO VERIFY BEFORE FLASHING -- these are placeholders, not
  measured values. Update them against your actual hardware.
  ---------------------------------------------------------------------------
  - 12 servos wired 3-per-leg (hip abduction, hip flexion, knee). The
    oscillator drives hip flexion + knee directly; hip abduction is held at
    a fixed stance-width trim angle (not part of the CPG state).
  - DS3218 Pro angle range and pulse width: this sketch uses Servo.write()
    in degrees (0-180 default range). If your calibration uses
    writeMicroseconds() instead, or your servos are extended-range,
    replace the write() calls in updateServos() accordingly.
  - IR break-beam gates wired to interrupt-capable pins, idle HIGH,
    pulled LOW on beam break (adjust FALLING/RISING if your wiring differs).
  - IMU (MPU-6050 now / BNO055 or ICM-20948 later) is NOT wired in here --
    plug your existing register-read or library code into readTiltDeg()
    and feed it into the RMS accumulator in the RUNNING state.
  - Pin numbers below are placeholders -- match them to your actual wiring.
*/

#include <Servo.h>

// ============================================================
// Leg / servo configuration
// ============================================================
const int NUM_LEGS = 4;
enum LegIndex { FL = 0, FR = 1, HL = 2, HR = 3 };

struct LegServos {
  int hipAbPin;
  int hipFlexPin;
  int kneePin;
  float hipAbTrimDeg;      // fixed stance-width angle, not driven by CPG
  float hipFlexCenterDeg;  // neutral hip angle the oscillator swings around
  float kneeCenterDeg;     // neutral knee angle the oscillator lifts from
};

// TODO: replace pins and center angles with your measured calibration
LegServos legs[NUM_LEGS] = {
  //  hipAb, hipFlex, knee,  trim,  hipCenter, kneeCenter
  {    2,      3,      4,   90.0,     90.0,      90.0 }, // FL
  {    5,      6,      7,   90.0,     90.0,      90.0 }, // FR
  {    8,      9,     10,   90.0,     90.0,      90.0 }, // HL
  {   11,     12,     13,   90.0,     90.0,      90.0 }, // HR
};

Servo hipAbServo[NUM_LEGS], hipFlexServo[NUM_LEGS], kneeServo[NUM_LEGS];

// ============================================================
// CPG state: phase (theta) and amplitude (r) per leg
// ============================================================
float theta[NUM_LEGS];   // radians
float r[NUM_LEGS];       // dimensionless, ramps 0 -> R_target

// ============================================================
// CPG parameters -- set per-trial over Serial with SET commands
// ============================================================
float nu = 1.0;        // stepping frequency, Hz          (your factor: Frequency)
float w  = 5.0;         // coupling weight                 (your factor: Coupling)
float R_target = 1.0;   // target oscillation amplitude
float a_conv = 50.0;    // amplitude convergence rate (1/s) -- higher = faster ramp-up

// Output scaling: how far the oscillator swings each joint, in degrees
float hipSwingDeg = 25.0;  // fore-aft hip excursion
float kneeLiftDeg = 30.0;  // knee lift during the swing half of the cycle

// ============================================================
// Gait phase-bias matrices
// Values are each leg's target phase as a FRACTION of one cycle,
// with FL as the reference (phase 0). Converted to radians on gait change.
// ============================================================
enum GaitType { WALK, TROT, PACE };
GaitType currentGait = TROT;

//                        FL      FR      HL      HR
float phaseBias[3][NUM_LEGS] = {
  { 0.00,   0.50,   0.25,   0.75 },  // WALK  -- sequential, statically stable
  { 0.00,   0.50,   0.50,   0.00 },  // TROT  -- diagonal pairs together
  { 0.00,   0.50,   0.00,   0.50 },  // PACE  -- same-side pairs together
};

float targetPhase[NUM_LEGS]; // radians, absolute target phase per leg

void setGait(GaitType g) {
  currentGait = g;
  for (int i = 0; i < NUM_LEGS; i++) {
    targetPhase[i] = phaseBias[g][i] * TWO_PI;
  }
}

// ============================================================
// Coupled oscillator derivatives (the CPG equations)
//
//   d(theta_i)/dt = 2*pi*nu + sum_j  w * sin(theta_j - theta_i - phi_ij)
//   d(r_i)/dt     = a_conv * (R_target - r_i)
//
// phi_ij is the desired steady-state value of (theta_j - theta_i), derived
// from each leg's absolute target phase: phi_ij = targetPhase[j] - targetPhase[i]
// ============================================================
void derivatives(float th[], float amp[], float dth[], float damp[]) {
  for (int i = 0; i < NUM_LEGS; i++) {
    float coupling = 0.0;
    for (int j = 0; j < NUM_LEGS; j++) {
      if (j == i) continue;
      float phi_ij = targetPhase[j] - targetPhase[i];
      coupling += w * sinf(th[j] - th[i] - phi_ij);
    }
    dth[i]  = TWO_PI * nu + coupling;
    damp[i] = a_conv * (R_target - amp[i]);
  }
}

// Classic 4th-order Runge-Kutta step. More accurate than Euler for the
// same step size, which matters here because oscillator drift compounds
// over a multi-second trial.
void rk4Step(float dt) {
  float k1_th[NUM_LEGS], k1_r[NUM_LEGS];
  float k2_th[NUM_LEGS], k2_r[NUM_LEGS];
  float k3_th[NUM_LEGS], k3_r[NUM_LEGS];
  float k4_th[NUM_LEGS], k4_r[NUM_LEGS];
  float tmp_th[NUM_LEGS], tmp_r[NUM_LEGS];

  derivatives(theta, r, k1_th, k1_r);

  for (int i = 0; i < NUM_LEGS; i++) {
    tmp_th[i] = theta[i] + 0.5f * dt * k1_th[i];
    tmp_r[i]  = r[i]     + 0.5f * dt * k1_r[i];
  }
  derivatives(tmp_th, tmp_r, k2_th, k2_r);

  for (int i = 0; i < NUM_LEGS; i++) {
    tmp_th[i] = theta[i] + 0.5f * dt * k2_th[i];
    tmp_r[i]  = r[i]     + 0.5f * dt * k2_r[i];
  }
  derivatives(tmp_th, tmp_r, k3_th, k3_r);

  for (int i = 0; i < NUM_LEGS; i++) {
    tmp_th[i] = theta[i] + dt * k3_th[i];
    tmp_r[i]  = r[i]     + dt * k3_r[i];
  }
  derivatives(tmp_th, tmp_r, k4_th, k4_r);

  for (int i = 0; i < NUM_LEGS; i++) {
    theta[i] += (dt / 6.0f) * (k1_th[i] + 2*k2_th[i] + 2*k3_th[i] + k4_th[i]);
    r[i]     += (dt / 6.0f) * (k1_r[i]  + 2*k2_r[i]  + 2*k3_r[i]  + k4_r[i]);

    // wrap phase to [0, 2*pi) so it doesn't grow unbounded over a long trial
    theta[i] = fmodf(theta[i], TWO_PI);
    if (theta[i] < 0) theta[i] += TWO_PI;
  }
}

// ============================================================
// Leg mapping: oscillator state -> joint angles
//
//   hip angle  tracks cos(theta)      -- fore-aft swing, symmetric
//   knee lift  tracks max(0, sin(theta)) -- lifts ONLY during swing phase,
//                                          so the leg stays planted in stance
// ============================================================
void updateServos() {
  for (int i = 0; i < NUM_LEGS; i++) {
    float hipAngle  = legs[i].hipFlexCenterDeg + hipSwingDeg * r[i] * cosf(theta[i]);
    float swingLift = fmaxf(0.0f, sinf(theta[i]));
    float kneeAngle = legs[i].kneeCenterDeg + kneeLiftDeg * r[i] * swingLift;

    hipFlexServo[i].write((int)hipAngle);
    kneeServo[i].write((int)kneeAngle);
    hipAbServo[i].write((int)legs[i].hipAbTrimDeg);
  }
}

// ============================================================
// Fixed-rate CPG integration via hardware timer
// 1 kHz is comfortably above your fastest stepping rate (2 Hz), which
// keeps RK4 accurate. Keep the ISR itself minimal -- it only sets a flag;
// the actual integration happens in loop().
// ============================================================
IntervalTimer cpgTimer;
const float CPG_DT = 0.001f; // seconds
volatile bool cpgTick = false;
void onCpgTimer() { cpgTick = true; }

// ============================================================
// Trial state machine (matches the protocol: skip the startup transient,
// then time the run) -- see the accompanying diagram
// ============================================================
enum TrialState { IDLE, STARTUP_TRANSIENT, RUNNING, COMPLETE };
TrialState trialState = IDLE;
unsigned long stateStartMs = 0;

// ~4-5 cycles of settling time, scaled by frequency so it's consistent
// across your 0.5-2.0 Hz range rather than a fixed wall-clock guess
unsigned long startupDurationMs() {
  return (unsigned long)(4.5f * (1000.0f / nu));
}

// ============================================================
// IR break-beam timing (interrupt-based -- timestamps the exact instant
// each beam is broken, not whenever the main loop happens to poll it)
// ============================================================
const int BEAM_START_PIN = 14;
const int BEAM_END_PIN   = 15;
volatile unsigned long beamStartMs = 0;
volatile unsigned long beamEndMs   = 0;
volatile bool beamStartTripped = false;
volatile bool beamEndTripped   = false;

void onBeamStart() {
  if (!beamStartTripped) {
    beamStartMs = millis();
    beamStartTripped = true;
  }
}
void onBeamEnd() {
  if (!beamEndTripped) {
    beamEndMs = millis();
    beamEndTripped = true;
  }
}

// ============================================================
// RMS tilt accumulator -- wire your IMU read into readTiltDeg().
// This just shows the accumulation pattern for the RUNNING state.
// ============================================================
float tiltSumSq = 0.0f;
int tiltSampleCount = 0;

float readTiltDeg() {
  // TODO: replace with your MPU-6050/BNO055 fused roll or pitch reading
  return 0.0f;
}

void accumulateTilt() {
  float t = readTiltDeg();
  tiltSumSq += t * t;
  tiltSampleCount++;
}

float rmsTiltDeg() {
  if (tiltSampleCount == 0) return 0.0f;
  return sqrtf(tiltSumSq / tiltSampleCount);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_LEGS; i++) {
    hipAbServo[i].attach(legs[i].hipAbPin);
    hipFlexServo[i].attach(legs[i].hipFlexPin);
    kneeServo[i].attach(legs[i].kneePin);
    theta[i] = (float)i * 0.01f; // tiny stagger so initial coupling isn't degenerate
    r[i] = 0.0f;                  // amplitude starts at 0 -- this IS the startup transient
  }

  setGait(TROT); // default; overwritten by the SET command before each trial

  pinMode(BEAM_START_PIN, INPUT_PULLUP);
  pinMode(BEAM_END_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BEAM_START_PIN), onBeamStart, FALLING);
  attachInterrupt(digitalPinToInterrupt(BEAM_END_PIN), onBeamEnd, FALLING);

  cpgTimer.begin(onCpgTimer, CPG_DT * 1e6f); // IntervalTimer takes microseconds
}

// ============================================================
// Serial command parser
//   SET nu=1.0 w=5 gait=TROT R=1.0     -- configure the next trial
//   RUN                                 -- start it (begins startup transient)
//   STOP                                -- abort back to IDLE
// Note: uses the Arduino String class for simplicity. For a long unattended
// session (540 trials), watch for heap fragmentation -- if you see erratic
// behavior after many trials, switch this parser to a fixed char buffer.
// ============================================================
void parseCommand(String line) {
  line.trim();

  if (line == "RUN") {
    trialState = STARTUP_TRANSIENT;
    stateStartMs = millis();
    beamStartTripped = false;
    beamEndTripped = false;
    tiltSumSq = 0.0f;
    tiltSampleCount = 0;
    for (int i = 0; i < NUM_LEGS; i++) r[i] = 0.0f; // re-trigger amplitude ramp-up
    return;
  }
  if (line == "STOP") {
    trialState = IDLE;
    return;
  }

  int start = 0;
  while (start < (int)line.length()) {
    int sp = line.indexOf(' ', start);
    if (sp == -1) sp = line.length();
    String token = line.substring(start, sp);
    int eq = token.indexOf('=');
    if (eq != -1) {
      String key = token.substring(0, eq);
      String val = token.substring(eq + 1);
      if      (key == "nu")   nu = val.toFloat();
      else if (key == "w")    w = val.toFloat();
      else if (key == "R")    R_target = val.toFloat();
      else if (key == "gait") {
        if      (val == "WALK") setGait(WALK);
        else if (val == "TROT") setGait(TROT);
        else if (val == "PACE") setGait(PACE);
      }
    }
    start = sp + 1;
  }
  Serial.println("OK");
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    parseCommand(line);
  }

  if (cpgTick) {
    cpgTick = false;
    rk4Step(CPG_DT);
  }

  // Servo + trial-state update at ~50 Hz: fast enough for smooth motion,
  // slow enough to leave headroom for serial and sensor work
  static unsigned long lastServoMs = 0;
  if (millis() - lastServoMs >= 20) {
    lastServoMs = millis();
    updateServos();

    switch (trialState) {
      case STARTUP_TRANSIENT:
        if (millis() - stateStartMs >= startupDurationMs()) {
          trialState = RUNNING;
          stateStartMs = millis();
        }
        break;

      case RUNNING:
        accumulateTilt();
        if (beamStartTripped && beamEndTripped) {
          float elapsedS = (beamEndMs - beamStartMs) / 1000.0f;
          Serial.print("TRIAL_RESULT time_s=");
          Serial.print(elapsedS, 4);
          Serial.print(" rms_tilt_deg=");
          Serial.println(rmsTiltDeg(), 4);
          trialState = COMPLETE;
        }
        break;

      case COMPLETE:
        // Host reads the TRIAL_RESULT line, logs it, and sends the next
        // trial's SET + RUN -- this is the loop back to IDLE
        trialState = IDLE;
        break;

      default:
        break;
    }
  }
}
