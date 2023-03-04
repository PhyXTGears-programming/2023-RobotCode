#pragma once

class Vector {
public:
    Vector(float x, float y, float z) : x(x), y(y), z(z) {}
    Vector() { // Heehoo Default Constructor
        x = 0;
        y = 0;
        z = 0;
    }

    float dot(Vector & rhs) {
        return (x * rhs.x) + (y * rhs.y) + (z * rhs.z);
    }

    float x;
    float y;
    float z;
};