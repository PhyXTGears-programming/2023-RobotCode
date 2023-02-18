#include "Mandatory.h"
#include "subsystems/arm/arm.h"

#include <numbers>
#include <cmath>

Point ArmSubsystem::calcElbowPos(double turretAng, double shoulderAng) {
    Point pt(
        Constants::Arm::k_bicepLenInches * std::cos(shoulderAng) * std::cos(turretAng),
        Constants::Arm::k_bicepLenInches * std::cos(shoulderAng) * std::sin(turretAng),
        Constants::Arm::k_bicepLenInches * std::sin(shoulderAng),
    );

    return pt;
}

Point ArmSubsystem::calcWristPos(
    double turretAng,
    double shoulderAng,
    double elbowAng
) {
    Point elbowPos = calcElbowPos(shoulderAng);
    Point pt(
        Constants::Arm::k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::cos(turretAng),
        Constants::Arm::k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::sin(turretAng),
        Constants::Arm::k_forearmLenInches * std::sin(shoulderAng + elbowAng)
    );

    return Point(elbowPos.x + pt.x, elbowPos.y + pt.y, elbowPos.z + pt.z);
}

ArmPose ArmSubsystem::calcIKJointPoses(Point pt) {
    double targetLen =  std::sqrt((std::pow(pt.x, 2) + std::pow(pt.y, 2) + std::pow(pt.z, 2))); // Line from shoulder to target
    double targetToXAxisAng = atan2(pt.z, std::sqrt(pt.x * pt.x + pt.y * pt.y));

    double targetToBicepAng = std::acos(
        (Constants::Arm::k_forearmLenInches * Constants::Arm::k_forearmLenInches
        - Constants::Arm::k_bicepLenInches * Constants::Arm::k_bicepLenInches
        + targetLen * targetLen)
        / (2.0 * Constants::Arm::k_forearmLenInches * targetLen)
    );

    // Solve IK:
    double bicepToXAxisAng = targetToBicepAng + targetToXAxisAng;

    double bicepToForearmAng =
        (std::numbers::pi / 2.0)
        - std::acos(
            (Constants::Arm::k_forearmLenInches * Constants::Arm::k_forearmLenInches
            - Constants::Arm::k_bicepLenInches * Constants::Arm::k_bicepLenInches
            + targetLen * targetLen)
            / (2.0 * Constants::Arm::k_forearmLenInches * targetLen));

    double turretToXZAng = std::atan2(pt.z, pt.x);

    return ArmPose(turretToXZAng, bicepToXAxisAng, bicepToForearmAng);

}

// Getting and setting arm angles:

units::turn_t ArmSubsystem::getShoulderAngle() {
    return mShoulderAngleSensor.Get();
}

double ArmSubsystem::getElbowAngle() {
    return mElbowAngleSensor.Get();
}

double ArmSubsystem::getTurretAngle() {
    return mTurretAngleSensor.Get();
}

double ArmSubsystem::getWristRollAngle() {
    return mWristRollAngleSensor.Get();
}

void ArmSubsystem::setTurretAngle(double angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setShoulderAngle(double angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setElbowAngle(double angle) {
    //NOT YET IMPLEMENTED
}

void ArmSubsystem::setWristRollAngle(double angle) {
    //you get the gist
}

//          NOT IMPLEMENTED IN HARDWARE
// float ArmSubsystem::getWristPitchAngle() {
//     return 0; // TODO when we get reading components available
// }
