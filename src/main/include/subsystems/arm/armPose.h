#pragma once

class ArmPose {
    public:
        ArmPose(double turretAngle, double shoulderAngle, double elbowAngle)
            : turretAngle(turretAngle),
            shoulderAngle(shoulderAngle),
            elbowAngle(elbowAngle) {}

        double turretAngle;
        double shoulderAngle;
        double elbowAngle;
};
