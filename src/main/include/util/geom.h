#pragma once

#include "util/point.h"
#include "util/vector.h"

Vector operator- (Point const & lhs, Point const & rhs);

Point operator+ (Point const & lhs, Vector const & rhs);
Point operator- (Point const & lhs, Vector const & rhs);

Vector operator* (Vector const &, double);
Vector operator* (double, Vector const &);