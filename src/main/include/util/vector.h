#pragma once

#include "util/math.h"

#include <cmath>

class Vector {
public:
    Vector(double x, double y, double z) : x(x), y(y), z(z) {}
    Vector() { // Heehoo Default Constructor
        x = 0;
        y = 0;
        z = 0;
    }

    double dot(Vector & rhs) {
        return (x * rhs.x) + (y * rhs.y) + (z * rhs.z);
    }

    double len() {
        return std::sqrt(x * x + y * y + z * z);
    }

    Vector unit() {
        double m = len();

        if (isNearZero(m)) {
            return Vector(1.0, 0.0, 0.0);
        } else {
            return Vector(x / m, y / m, z / m);
        }
    }

    double x;
    double y;
    double z;
};

Vector operator* (Vector const &, double);
Vector operator* (double, Vector const &);
