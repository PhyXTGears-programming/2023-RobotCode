#pragma once

#include "armPose.h"
#include "point.h"
#include "boundary.h"

#include <cmath>

#include <frc2/command/SubsystemBase.h>

class ArmSubsystem : public frc2::SubsystemBase {
public:

    // Calulating Functions:
    Point calcElbowPos(float angShoulder);

    Point calcWristPos(float shoulderAng, float elbowAng);

    ArmPose calcIKJointPoses(Point pt);

    bool checkBoundary(Boundary boundary, Point point);


    // Reading Functions:
    float getShoulderAngle();

    float getElbowAngle();

    float getTurretAngle();

private:

};