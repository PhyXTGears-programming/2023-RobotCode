#include "util/math.h"

bool isNearZero(double val, double tolerance) {
    return -tolerance < val && val < tolerance;
}