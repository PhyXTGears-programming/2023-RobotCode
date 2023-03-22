#include "subsystems/arm/motionPath.h"

#include <algorithm>

MotionPath::MotionPath(std::vector<Point> path) : m_path(path) {
    std::reverse(m_path.begin(), m_path.end());
    c_finalTarget = m_path[0];
}

/// Yield next point in path after each call.
Point MotionPath::getNextPoint() {
    m_target = m_path.back();
    m_path.pop_back();

    return m_target;
}

Point MotionPath::interpolate(Point const & p0, Point const & p1) {
    return Point(p0);
}

bool MotionPath::isDone() {
    return m_path.size() == 1 && c_finalTarget.isNear(m_path[0]);
}
