#include "util/geom.h"

Vector operator-(const Point & lhs, const Point & rhs) {
    return Vector(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

Point operator+(const Point & lhs, const Vector & rhs) {
    return Point{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
}

Point operator-(const Point & lhs, const Vector & rhs) {
    return Point{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
}

Vector operator*(const Vector & a, double b) {
    return Vector(a.x * b, a.y * b, a.z * b);
}

Vector operator*(double a, const Vector & b) {
    return Vector(a * b.x, a * b.y, a * b.z);
}
