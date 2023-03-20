#include "subsystems/arm/motionPath.h"

MotionPath::MotionPath(std::vector<Point> path) : m_path(path) {
    m_finalTarget = m_path[m_path.size() - 1];
}

Point MotionPath::getNextPoint() {
    // FIXME: return the next step in the path.
    return m_finalTarget;
}

Point MotionPath::interpolate(Point const & p0, Point const & p1) {
    return Point(p0);
}

bool MotionPath::isDone() {
    return m_path.size() == 1 && m_finalTarget.isNear(m_path[0]);
}