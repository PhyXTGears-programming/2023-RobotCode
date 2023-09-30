#pragma once

#include "util/vector.h"

#include <cmath>

static const double NEAR_ZERO_METERS = 0.01;

class Point {
public:
    Point(double x, double y)
        : x(x)
        , y(y) {}
    Point() { // default constructor
        x = 0;
        y = 0;
    } // default constructor

    double x;
    double y;

    bool isNear(const Point & rhs) const {
        return std::abs(x - rhs.x) < NEAR_ZERO_METERS && std::abs(y - rhs.y) < NEAR_ZERO_METERS;
    }
};
