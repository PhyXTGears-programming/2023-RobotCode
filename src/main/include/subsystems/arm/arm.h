#pragma once

#include "armPose.h"
#include "point.h"

#include <cmath>

#include <frc2/command/SubsystemBase.h>

class ArmSubsystem : public frc2::SubsystemBase {
    public:

        Point getElbowPos(float angShoulder);

        Point getWristPos(float shoulderAng, float elbowAng);

        ArmPose getIKJointPoses(Point pt);

private:

};