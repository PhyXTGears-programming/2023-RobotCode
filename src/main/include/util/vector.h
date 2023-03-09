#pragma once

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

    double x;
    double y;
    double z;
};
