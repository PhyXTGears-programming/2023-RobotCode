#include <gtest/gtest.h>

#include "util/point.h"
#include "subsystems/arm/boundary.h"

TEST(BoxBoundaryTest, GivenPointInsideBox_IsInsideReturnsTrue) {
    BoxBoundary boundary {
        -1.0, 1.0,
        -1.0, 1.0,
        -1.0, 1.0
    };

    Point insidePoint {0.0, 0.0, 0.0};

    ASSERT_TRUE(boundary.isInside(insidePoint));
    ASSERT_FALSE(boundary.isOutside(insidePoint));
}

TEST(BoxBoundaryTest, GivenPointOutsideBox_IsInsideReturnsFalse) {
    BoxBoundary boundary {
        -1.0, 1.0,
        -1.0, 1.0,
        -1.0, 1.0
    };

    Point outsidePoint {2.0, 0.0, 0.0};

    ASSERT_FALSE(boundary.isInside(outsidePoint));
    ASSERT_TRUE(boundary.isOutside(outsidePoint));
}

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

TEST(SphereBoundaryTest, GivenPointInsideSphere_IsInsideReturnsTrue) {
    SphereBoundary boundary {
        Point{0.0, 0.0, 0.0},
        1.0
    };

    Point insidePoint {0.0, 0.0, 0.5};

    ASSERT_TRUE(boundary.isInside(insidePoint));
    ASSERT_FALSE(boundary.isOutside(insidePoint));
}

TEST(SphereBoundaryTest, GivenPointOutsideSphere_IsInsideReturnsFalse) {
    SphereBoundary boundary {
        Point{0.0, 0.0, 0.0},
        1.0
    };

    Point outsidePoint {2.0, 0.0, 0.0};

    ASSERT_FALSE(boundary.isInside(outsidePoint));
    ASSERT_TRUE(boundary.isOutside(outsidePoint));
}
