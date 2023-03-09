#include "util/vector.h"

Vector operator*(Vector const & a, double b) {
    return Vector(a.x * b, a.y * b, a.z * b);
}

Vector operator*(double a, Vector const & b) {
    return Vector(a * b.x, a * b.y, a * b.z);
}
