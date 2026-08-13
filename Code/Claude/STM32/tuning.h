// ─────────────────────────────────────────────
//  tuning.h / tuning.cpp  –  Serial PID tuner
//
//  Connect via USB-serial (ST-Link VCP) at 115200.
//  Commands are single-line ASCII, terminated with \n
//
//  COMMAND FORMAT:
//  <target>.<param>=<value>
//
//  Targets:
//    ip  = inner PID
//    op  = outer PID
//    lqr = LQR gains
//    fil = filter
//    gait= gait params
//
//  Params for PID: kp, ki, kd, max
//  Params for LQR: k1, k2, k3, k4
//  Params for filter: alpha, type (0=comp, 1=madgwick)
//  Params for gait: speed, step
//
//  EXAMPLES:
//    ip.kp=85.0       → set inner P gain to 85
//    op.ki=0.05       → set outer I gain to 0.05
//    lqr.k1=50.0      → set LQR tilt gain
//    fil.type=1       → switch to Madgwick filter
//    gait.speed=0.7   → set walk speed to 70%
//    ?                → print all current values
//    save             → save to flash (future feature)
//    mode=walk        → switch to walk mode
//    mode=drive       → switch to drive mode
// ─────────────────────────────────────────────
#pragma once

void Tuning_Init(void);
void Tuning_Process(void);      // call every main loop iteration
void Tuning_PrintAll(void);     // print all current gains over serial
