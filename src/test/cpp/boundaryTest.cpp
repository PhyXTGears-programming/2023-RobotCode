#include <gtest/gtest.h>

#include "util/point.h"
#include "subsystems/arm/boundary.h"

TEST(ConvexPolygonBoundaryTest, GivenPointInsideTriangle_IsInsideReturnsTrue) {
    ConvexPolygonBoundary boundary {
        {
            Point{0.0, 1.0, 0.0},
            Point{-1.0, -1.0, 0.0},
            Point{1.0, -1.0, 0.0}
        },
        0.0, 1.0
    };

    Point inTrianglePoint {0.0, 0.0, 0.5};

    ASSERT_TRUE(boundary.isInside(inTrianglePoint));
    ASSERT_FALSE(boundary.isOutside(inTrianglePoint));
}

TEST(ConvexPolygonBoundaryTest, GivenPointOutsideTriangle_IsInsideReturnsFalse) {
    ConvexPolygonBoundary boundary {
        {
            Point{0.0, 1.0, 0.0},
            Point{-1.0, -1.0, 0.0},
            Point{1.0, -1.0, 0.0}
        },
        0.0, 1.0
    };

    Point inTrianglePoint {0.0, 2.0, 0.5};

    ASSERT_FALSE(boundary.isInside(inTrianglePoint));
    ASSERT_TRUE(boundary.isOutside(inTrianglePoint));
}