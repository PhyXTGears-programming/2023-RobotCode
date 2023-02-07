#pragma once

#include "arm.h"

#include <vector>

class Boundary {
    public:
        bool isInside(Point pt);
        bool isOutside(Point pt);
};

class BoxBoundary : public Boundary {
public:
    BoxBoundary(
        float xLo,
        float xHi,
        float yLo,
        float yHi,
        float zLo,
        float zHi)
    : xLo(xLo), xHi(xHi), yLo(yLo), yHi(yHi), zLo(zLo), zHi(zHi) { }
    
    bool isInside(Point pt) {
        return (xLo < pt.x && pt.x < xHi)
            && (yLo < pt.y && pt.y < yHi)
            && (zLo < pt.z && pt.z < zHi);
    }

    bool isOutside(Point pt) {
        return !isInside(pt);
    }

private:
    float xLo;
    float xHi;
    float yLo;
    float yHi;
    float zLo;
    float zHi;
};

class SphereBoundary : public Boundary {
public:
    SphereBoundary(Point center, float radius) : center(center), radius(radius) { }

    bool isInside(Point pt) {
        Point offset((center.x - pt.x), (center.y - pt.y), (center.z - pt.z));

        float dist =
               (offset.x * offset.x)
             + (offset.y * offset.y)
             + (offset.z * offset.z);

        return (dist < radius * radius);
    }

    bool isOutside(Point pt) {
        return !isInside(pt);
    }

private:
    Point center;
    float radius;
};

class ComposeBoundary : public Boundary {
public:
    ComposeBoundary(std::vector<std::unique_ptr<Boundary>> boundaries)
        : boundaries(boundaries) { }

    bool isInsideBounds(Point pt) {
        for (auto& bound : boundaries) {
            if (bound->isOutside(pt)) {
                return false;
            }
        }

        return true;
    }

    bool isOutsideBounds(Point pt) {
        return !isInsideBounds(pt);
    }

private:
    std::vector<std::unique_ptr<Boundary>> boundaries;
};

class NotBoundary : public Boundary {
public:
    bool isInsideBounds(Point pt) {
        return mBound->isOutside(pt);
    }

    bool isOutsideBounds(Point pt) {
        return !isInsideBounds(pt);
    }

private:
    std::unique_ptr<Boundary> mBound;
};
