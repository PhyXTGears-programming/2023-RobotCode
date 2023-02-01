#pragma once

#include "armPose.h"
#include "point.h"
#include "boundary.h"

#include <cmath>

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/DutyCycleEncoder.h>

class ArmSubsystem : public frc2::SubsystemBase {
public:

    // Calulating Functions:
    Point calcElbowPos(float angShoulder);

    Point calcWristPos(float shoulderAng, float elbowAng);

    ArmPose calcIKJointPoses(Point pt);

    bool checkBoundary(Boundary boundary, Point point);


    // Reading Functions:
    float getTurretAngle();

    units::turn_t getShoulderAngle();

    float getElbowAngle();

    float getWristRollAngle();

    // float getWristPitchAngle(); // NOT IMPLEMENTED IN HARDWARE

private:

    frc::AnalogPotentiometer mTurretAngleSensor {1, 1.0, 0};

    frc::DutyCycleEncoder mShoulderAngleSensor{1}; // Using Funky Fresh Encoder

    frc::AnalogPotentiometer mElbowAngleSensor {1, 1.0, 0};

    frc::AnalogPotentiometer mWristAngleSensor {1, 1.0, 0};

};
