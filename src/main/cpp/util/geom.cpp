#include "util/geom.h"

Vector operator- (Point const & lhs, Point const & rhs) {
    return Vector(
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z
    );
}

Point operator+ (Point const & lhs, Vector const & rhs) {
    return Point {
        lhs.x + rhs.x,
        lhs.y + rhs.y,
        lhs.z + rhs.z
    };
}

Point operator- (Point const & lhs, Vector const & rhs) {
    return Point {
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z
    };
}

Vector operator*(Vector const & a, double b) {
    return Vector(a.x * b, a.y * b, a.z * b);
}

Vector operator*(double a, Vector const & b) {
    return Vector(a * b.x, a * b.y, a * b.z);
}
