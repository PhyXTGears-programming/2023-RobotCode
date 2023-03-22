#pragma once

#include "util/geom.h"

#include <vector>

class MotionPath {
public:
    MotionPath(std::vector<Point> path);

    Point interpolate(Point const & pt0, Point const & pt1);

    Point getNextPoint();

    bool isDone();

private:
    std::vector<Point> m_path;

    Point c_finalTarget;
    Point m_target;
};
