#pragma once

#include "util/vector.h"

#include <cmath>

static const double NEAR_ZERO_METERS = 0.005;

class Point {
    public:
        Point(float x, float y, float z) : x(x), y(y), z(z) {}
        Point() { // default constructor
            x = 0;
            y = 0;
            z = 0;
        } // default constructor

        float x;
        float y;
        float z;


    Vector operator- (Point const & rhs) const {
        return Vector(
            x - rhs.x,
            y - rhs.y,
            z - rhs.z
        );
    }

    bool isNear(Point const & rhs) const {
        return std::abs(x - rhs.x) < NEAR_ZERO_METERS
            && std::abs(y - rhs.y) < NEAR_ZERO_METERS
            && std::abs(z - rhs.z) < NEAR_ZERO_METERS;
    }
};
