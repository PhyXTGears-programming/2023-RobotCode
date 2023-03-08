#pragma once

class ArmPose {
    public:
        ArmPose(float turretAngle, float shoulderAngle, float elbowAngle)
            : turretAngle(turretAngle),
            shoulderAngle(shoulderAngle),
            elbowAngle(elbowAngle) {}

        float turretAngle;
        float shoulderAngle;
        float elbowAngle;
};
