#pragma once

#include "util/point.h"

#include <vector>

class MotionPath {
public:
    MotionPath(std::vector<Point> path) : m_path(path) {
        m_finalTarget = m_path[m_path.size() - 1];
    }

    Point interpolate(Point const & pt0, Point const & pt1);

    Point getNextPoint();

    bool isDone();

    // ArmSubsystem::isNearPoint(m_target) && m_path.size() == 1;

private:
    std::vector<Point> m_path;

    Point m_finalTarget;
    Point m_target;
};

// command:
//     MotionPath path;
//     Point finalTarget;
//     Point target = path.getNextPoint();
//     ArmSubsystem arm;
//     loop {
//         if (finalTarget.isNearPoint(target) && arm.isNearPoint(target)) {
//             break;
//         } else if (arm.isNearPoint(target)) {
//             target = path.getNextPoint();
//         }

//         use ik to get pose
//         apply to subsystem

//     }


// command:
//     MotionPath path;
//     Point finalTarget;
//     ArmSubsystem arm;

//     loop {
//         if (path.isDone()) {
//             break;
//         } else {
//             point = path.getNextPoint();

//             usee ik to get pose
//             apply to subsystem.
//         }
//     }