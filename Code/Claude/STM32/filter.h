#pragma once
// ─────────────────────────────────────────────
//  filter.h  –  Attitude filter interface
// ─────────────────────────────────────────────
#include "imu.h"

// Choose which filter to use at runtime
enum FilterType { FILTER_COMPLEMENTARY, FILTER_MADGWICK };

void    Filter_Init(FilterType type, float dt);
void    Filter_Update(const IMUData* imu, Attitude* att);
void    Filter_Reset(void);
void    Filter_SetType(FilterType type);
