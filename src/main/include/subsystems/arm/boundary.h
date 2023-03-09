#pragma once

#include "util/geom.h"

#include <memory>
#include <vector>

class Boundary {
    public:
        virtual bool isInside(Point const & pt) { return true; }
        virtual bool isOutside(Point const & pt) { return true; }

};

class BoxBoundary : public Boundary {
public:
    BoxBoundary(
        double xLo,
        double xHi,
        double yLo,
        double yHi,
        double zLo,
        double zHi)
    : xLo(xLo), xHi(xHi), yLo(yLo), yHi(yHi), zLo(zLo), zHi(zHi) { }
    
    bool isInside(Point const & pt) override {
        return (xLo < pt.x && pt.x < xHi)
            && (yLo < pt.y && pt.y < yHi)
            && (zLo < pt.z && pt.z < zHi);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    double xLo;
    double xHi;
    double yLo;
    double yHi;
    double zLo;
    double zHi;
};

// Polygon points must wind counter-clockwise.
class ConvexPolygonBoundary : public Boundary {
public:
    ConvexPolygonBoundary(std::vector<Point> points, double zLo, double zHi)
    : points(points), zLo(zLo), zHi(zHi) {
        // Visit all points except the last point.
        // Compute vector to following point.
        for (unsigned int i = 0; i < points.size() - 1; i++) {
            Vector v = points[i + 1] - points[i];
            vectors.push_back(Vector{-v.y, v.x, v.z});
        }

        // Visit the last point.
        // Compute vector back to first point.
        vectors.push_back(points[0] - points[points.size() - 1]);
    }

    bool isInside(Point const & pt) override {
        std::vector<Vector> targets;

        for (unsigned int i = 0; i < points.size(); i++) {
            targets.push_back(pt - points[i]);
        }

        for (unsigned int i = 0; i < vectors.size(); i++) {
            if (vectors[i].dot(targets[i]) < 0.0) {
                return false;
            }
        }

        return true;
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    std::vector<Point> points;
    std::vector<Vector> vectors;
    double zLo;
    double zHi;
};

class CylinderBoundary : public Boundary {
public:
    CylinderBoundary(Point center, double radius, double zLo, double zHi)
    : center(center), radius(radius), zLo(zLo), zHi(zHi) {}

    bool isInside(Point const & pt) override {
        // Check XY plane
        Point offset((center.x - pt.x), (center.y - pt.y), 0.0);
        double dist =
          (offset.x * offset.x)
        + (offset.y * offset.y);

        if (dist < radius * radius) {
            return zLo < pt.z && pt.z < zHi;
        } else {
            return true;
        }
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    Point center;
    double radius;
    double zLo;
    double zHi;
};

class SphereBoundary : public Boundary {
public:
    SphereBoundary(Point center, double radius) : center(center), radius(radius) { }

    bool isInside(Point const & pt) override {
        Point offset((center.x - pt.x), (center.y - pt.y), (center.z - pt.z));

        double dist =
               (offset.x * offset.x)
             + (offset.y * offset.y)
             + (offset.z * offset.z);

        return (dist < radius * radius);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    Point center;
    double radius;
};

class ComposeBoundary : public Boundary {
public:
    ComposeBoundary(std::vector<std::shared_ptr<Boundary>> && boundaries)
        : boundaries(boundaries) { }

    bool isInside(Point const & pt) override {
        for (auto& bound : boundaries) {
            if (bound->isInside(pt)) {
                return true;
            }
        }

        return false;
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    std::vector<std::shared_ptr<Boundary>> boundaries;
};

class NotBoundary : public Boundary {
public:

    NotBoundary(std::unique_ptr<Boundary> boundary) : m_Bound(std::move(boundary)) {}

    bool isInside(Point const & pt) override {
        return m_Bound->isOutside(pt);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    std::unique_ptr<Boundary> m_Bound;
};
