#include "Mandatory.h"
#include "subsystems/arm/arm.h"

#include <numbers>
#include <cmath>

Point ArmSubsystem::calcElbowPos(float angShoulder) {
    Point pt(
        k_bicepLenInches * std::cos(angShoulder),
        k_bicepLenInches * std::sin(angShoulder),
        0.0
    );

    return pt;
}

Point ArmSubsystem::calcWristPos(float shoulderAng, float elbowAng) {
    Point elbowPos = calcElbowPos(shoulderAng);
    Point pt(
        k_forearmLenInches * std::cos(shoulderAng + elbowAng),
        k_forearmLenInches * std::sin(shoulderAng + elbowAng),
        0.0
    );

    return Point(elbowPos.x + pt.x, elbowPos.y + pt.y, 0.0);
}

ArmPose ArmSubsystem::calcIKJointPoses(Point pt) {
    float targetLen =  std::sqrt((pt.x * pt.x + pt.y * pt.y)); // Line from shoulder to target
    float targetToXAxisAng = std::atan2(pt.y, pt.x);

    float targetToBicepAng = std::acos(
        (k_forearmLenInches * k_forearmLenInches 
        - k_bicepLenInches * k_bicepLenInches
        + targetLen * targetLen)
        / (2.0 * k_forearmLenInches * targetLen)
    );

    // Solve IK:
    float bicepToXAxisAng = targetToBicepAng + targetToXAxisAng;

    float bicepToForearmAng =
        (std::numbers::pi / 2.0)
        - std::acos(
            (k_forearmLenInches * k_forearmLenInches
            - k_bicepLenInches * k_bicepLenInches
            + targetLen * targetLen)
            / (2.0 * k_forearmLenInches * targetLen));

    float turretToXZAng = std::atan2(pt.z, pt.x);

    return ArmPose(turretToXZAng, bicepToXAxisAng, bicepToForearmAng);

}

// Getting and setting arm angles:

units::turn_t ArmSubsystem::getShoulderAngle() {
    return mShoulderAngleSensor.Get();
}

float ArmSubsystem::getElbowAngle() {
    return mElbowAngleSensor.Get();
}

float ArmSubsystem::getTurretAngle() {
    return mTurretAngleSensor.Get();
}

float ArmSubsystem::getWristRollAngle() {
    return mWristRollAngleSensor.Get();
}

void ArmSubsystem::setTurretAngle(float angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setShoulderAngle(float angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setElbowAngle(float angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setWristRollAngle(float angle) {
    //you get the gist
}

//          NOT IMPLEMENTED IN HARDWARE
// float ArmSubsystem::getWristPitchAngle() {
//     return 0; // TODO when we get reading components available
// }
