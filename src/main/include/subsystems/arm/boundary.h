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
    : c_xLo(xLo), c_xHi(xHi), c_yLo(yLo), c_yHi(yHi), c_zLo(zLo), c_zHi(zHi) { }

    bool isInside(Point const & pt) override {
        return (c_xLo < pt.x && pt.x < c_xHi)
            && (c_yLo < pt.y && pt.y < c_yHi)
            && (c_zLo < pt.z && pt.z < c_zHi);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    double c_xLo;
    double c_xHi;
    double c_yLo;
    double c_yHi;
    double c_zLo;
    double c_zHi;
};

// Polygon points must wind counter-clockwise.
class ConvexPolygonBoundary : public Boundary {
public:
    ConvexPolygonBoundary(std::vector<Point> points, double zLo, double zHi)
    : c_points(points), c_zLo(zLo), c_zHi(zHi) {
        // Visit all points except the last point.
        // Compute vector to following point.
        for (unsigned int i = 0; i < points.size() - 1; i++) {
            Vector v = points[i + 1] - points[i];
            c_vectors.push_back(Vector{-v.y, v.x, v.z});
        }

        // Visit the last point.
        // Compute vector back to first point.
        c_vectors.push_back(points[0] - points[points.size() - 1]);
    }

    bool isInside(Point const & pt) override {
        std::vector<Vector> targets;

        for (unsigned int i = 0; i < c_points.size(); i++) {
            targets.push_back(pt - c_points[i]);
        }

        for (unsigned int i = 0; i < c_vectors.size(); i++) {
            if (c_vectors[i].dot(targets[i]) < 0.0) {
                return false;
            }
        }

        return c_zLo < pt.z && pt.z < c_zHi;
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    std::vector<Point> c_points;
    std::vector<Vector> c_vectors;
    double c_zLo;
    double c_zHi;
};

class CylinderBoundary : public Boundary {
public:
    CylinderBoundary(Point center, double radius, double zLo, double zHi)
    : c_center(center), c_radius(radius), c_zLo(zLo), c_zHi(zHi) {}

    bool isInside(Point const & pt) override {
        // Check XY plane
        Point offset((c_center.x - pt.x), (c_center.y - pt.y), 0.0);
        double dist =
          (offset.x * offset.x)
        + (offset.y * offset.y);

        if (dist < c_radius * c_radius) {
            return c_zLo < pt.z && pt.z < c_zHi;
        } else {
            return false;
        }
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    Point c_center;
    double c_radius;
    double c_zLo;
    double c_zHi;
};

class SphereBoundary : public Boundary {
public:
    SphereBoundary(Point center, double radius) : c_center(center), c_radius(radius) { }

    bool isInside(Point const & pt) override {
        Point offset((c_center.x - pt.x), (c_center.y - pt.y), (c_center.z - pt.z));

        double dist =
               (offset.x * offset.x)
             + (offset.y * offset.y)
             + (offset.z * offset.z);

        return (dist < c_radius * c_radius);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    Point c_center;
    double c_radius;
};

class ComposeBoundary : public Boundary {
public:
    ComposeBoundary(std::vector<std::shared_ptr<Boundary>> && boundaries)
        : c_boundaries(boundaries) { }

    bool isInside(Point const & pt) override {
        for (auto& bound : c_boundaries) {
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
    std::vector<std::shared_ptr<Boundary>> c_boundaries;
};

class NotBoundary : public Boundary {
public:

    NotBoundary(std::unique_ptr<Boundary> boundary) : c_bound(std::move(boundary)) {}

    bool isInside(Point const & pt) override {
        return c_bound->isOutside(pt);
    }

    bool isOutside(Point const & pt) override {
        return !isInside(pt);
    }

private:
    std::unique_ptr<Boundary> c_bound;
};
