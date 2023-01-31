#include "Mandatory.h"
#include "subsystems/arm/arm.h"

#include <numbers>
#include <cmath>

Point ArmSubsystem::getElbowPos(float angShoulder) {
    Point pt(
        k_bicepLenInches * std::cos(angShoulder),
        k_bicepLenInches * std::sin(angShoulder),
        0.0
    );

    return pt;
}

Point ArmSubsystem::getWristPos(float shoulderAng, float elbowAng) {
    Point elbowPos = getElbowPos(shoulderAng);
    Point pt(
        k_forearmLenInches * std::cos(shoulderAng + elbowAng),
        k_forearmLenInches * std::sin(shoulderAng + elbowAng),
        0.0
    );

    return Point(elbowPos.x + pt.x, elbowPos.y + pt.y, 0.0);
}

ArmPose ArmSubsystem::getIKJointPoses(Point pt) {
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

    return ArmPose(bicepToXAxisAng, bicepToForearmAng);

}
