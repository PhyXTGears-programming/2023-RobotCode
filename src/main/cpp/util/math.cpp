#include "util/math.h"

bool isNearZero(double val) {
    return -0.001 < val && val < 0.001;
}