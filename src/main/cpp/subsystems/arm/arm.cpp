#include "Mandatory.h"
#include "subsystems/arm/arm.h"

#include <numbers>
#include <cmath>

Point ArmSubsystem::calcElbowPos(float turretAng, float shoulderAng) {
    Point pt(
        k_bicepLenInches * std::cos(shoulderAng) * std::cos(turretAng),
        k_bicepLenInches * std::cos(shoulderAng) * std::sin(turretAng),
        k_bicepLenInches * std::sin(shoulderAng),
    );

    return pt;
}

Point ArmSubsystem::calcWristPos(
    float turretAng,
    float shoulderAng,
    float elbowAng
) {
    Point elbowPos = calcElbowPos(shoulderAng);
    Point pt(
        k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::cos(turretAng),
        k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::sin(turretAng),
        k_forearmLenInches * std::sin(shoulderAng + elbowAng)
    );

    return Point(elbowPos.x + pt.x, elbowPos.y + pt.y, elbowPos.z + pt.z);
}

ArmPose ArmSubsystem::calcIKJointPoses(Point pt) {
    float targetLen =  std::sqrt((std::pow(pt.x, 2) + std::pow(pt.y, 2) + std::pow(pt.z, 2))); // Line from shoulder to target
    float targetToXAxisAng = atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y));

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
