#pragma once

#include "arm.h"

#include <vector>

class Boundary {
    public:
        virtual bool isInside(Point pt);
        virtual bool isOutside(Point pt);
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
    
    bool isInside(Point pt) override {
        return (xLo < pt.x && pt.x < xHi)
            && (yLo < pt.y && pt.y < yHi)
            && (zLo < pt.z && pt.z < zHi);
    }

    bool isOutside(Point pt) override {
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

    bool isInside(Point pt) override {
        Point offset((center.x - pt.x), (center.y - pt.y), (center.z - pt.z));

        float dist =
               (offset.x * offset.x)
             + (offset.y * offset.y)
             + (offset.z * offset.z);

        return (dist < radius * radius);
    }

    bool isOutside(Point pt) override {
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

    bool isInside(Point pt) override {
        for (auto& bound : boundaries) {
            if (bound->isOutside(pt)) {
                return false;
            }
        }

        return true;
    }

    bool isOutside(Point pt) override {
        return !isInside(pt);
    }

private:
    std::vector<std::unique_ptr<Boundary>> boundaries;
};

class NotBoundary : public Boundary {
public:

    NotBoundary(std::unique_ptr<Boundary> boundary) : m_Bound(std::move(boundary)) {}

    bool isInside(Point pt) override {
        return m_Bound->isOutside(pt);
    }

    bool isOutside(Point pt) override {
        return !isInside(pt);
    }

private:
    std::unique_ptr<Boundary> m_Bound;
};
