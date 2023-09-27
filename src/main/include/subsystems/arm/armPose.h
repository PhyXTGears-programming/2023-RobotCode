#pragma once

class ArmPose {
public:
    ArmPose(double shoulderAngle, double elbowAngle)
        : shoulderAngle(shoulderAngle)
        , elbowAngle(elbowAngle) {}

    double shoulderAngle;
    double elbowAngle;
};
