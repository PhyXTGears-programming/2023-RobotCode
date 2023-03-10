#pragma once

// MACROS
#define RAD_2_DEG(rad) ((rad) * 180.0 / M_PI)
#define DEG_2_RAD(deg) ((deg) * M_PI / 180.0)

bool isNearZero(double val, double tolerance = 0.001);
