#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "util/math.h"

#include <numbers>
#include <cmath>
#include <sstream>

#include <frc/smartdashboard/SmartDashboard.h>

const double k_ChassisXSize = 0.775; // meters
const double k_ChassisYSize = 0.826; // meters
const double k_ChassisZSize = 0.229; // meters

const double k_PickupXSize = 0.076;  // meters
const double k_PickupYSize = 0.340;  // meters
const double k_PickupZSize = 1.000;  // meters

ArmSubsystem::ArmSubsystem(std::shared_ptr<cpptoml::table> toml) {
    loadConfig(toml);
}

void ArmSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Turret Angle", getTurretAngle());
    frc::SmartDashboard::PutNumber("Shoulder Angle", getShoulderAngle());
    frc::SmartDashboard::PutNumber("Elbow Angle", getElbowAngle());
    frc::SmartDashboard::PutNumber("Wrist Roll Angle", getWristRollAngle());
    frc::SmartDashboard::PutNumber("Grip Angle", getGrip());
}

void ArmSubsystem::initialiseBoundary() {
    // Only create boundaries for no-go zones.
    // Otherwise, the code gets really complicated later
    // to check for collisions.

    std::shared_ptr<Boundary> chassisNoGoBounds = std::make_shared<ComposeBoundary>(std::vector<std::shared_ptr<Boundary>>{
        std::make_shared<BoxBoundary>(
            -(k_ChassisXSize / 2.0), (k_ChassisXSize / 2.0),
             (k_PickupYSize / 2.0) , (k_ChassisYSize / 2.0),
              0.0                  , (k_ChassisZSize)
        ),
        std::make_shared<BoxBoundary>(
            -(k_ChassisXSize / 2.0), (k_ChassisXSize / 2.0) - k_PickupXSize,
            -(k_PickupYSize / 2.0) , (k_PickupYSize / 2.0),
              0.0                  , (k_ChassisZSize)
        ),
        std::make_shared<BoxBoundary>(
            -(k_ChassisXSize / 2.0),  (k_ChassisXSize / 2.0),
            -(k_ChassisYSize / 2.0), -(k_PickupYSize / 2.0),
              0.0                  ,  (k_ChassisZSize)
        )
    });

    //std::unique_ptr<> turretNoGoZone = std::make_unique<>

    std::shared_ptr<ComposeBoundary> defNoGoZone = std::make_shared<ComposeBoundary>(std::vector{
        std::move(chassisNoGoBounds)
    });

    m_noGoZone = std::move(defNoGoZone);
}

Point ArmSubsystem::calcElbowPos(double turretAng, double shoulderAng) {
    Point pt{
        Constants::Arm::k_bicepLenInches * std::cos(shoulderAng) * std::cos(turretAng),
        Constants::Arm::k_bicepLenInches * std::cos(shoulderAng) * std::sin(turretAng),
        Constants::Arm::k_bicepLenInches * std::sin(shoulderAng),
    };

    return pt;
}

Point ArmSubsystem::calcWristPos(
    double turretAng,
    double shoulderAng,
    double elbowAng
) {
    Point elbowPos = calcElbowPos(turretAng, shoulderAng);
    Point pt(
        Constants::Arm::k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::cos(turretAng),
        Constants::Arm::k_forearmLenInches * std::cos(shoulderAng + elbowAng) * std::sin(turretAng),
        Constants::Arm::k_forearmLenInches * std::sin(shoulderAng + elbowAng)
    );

    return Point(elbowPos.x + pt.x, elbowPos.y + pt.y, elbowPos.z + pt.z);
}

ArmPose ArmSubsystem::calcIKJointPoses(Point const & pt) {
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

bool ArmSubsystem::isPointSafe(Point const & point) {
    if (nullptr == m_noGoZone) {
        return false;
    } else {
        return m_noGoZone->isOutside(point);
    }
}

bool ArmSubsystem::isNearPoint(Point const & point) {
    Point currentPosition = calcWristPos(
        getTurretAngle(),
        getShoulderAngle(),
        getElbowAngle()
    );

    return point.isNear(currentPosition);
}

MotionPath ArmSubsystem::getPathTo(Point const & current, Point const & target) {
    // FIXME: compute proper path sequence for arm to follow.
    return MotionPath({ target });
}

//  ============  Point  Grabbers:  ============  //|

Point const & ArmSubsystem::getIntakePoint() {  //  | Intake
    return ArmSubsystem::m_pointIntake;
}
Point const & ArmSubsystem::getHomePoint() {    //  | Home
    return ArmSubsystem::m_pointHome;
}
Point const & ArmSubsystem::getHybridPoint() {  //  | Hybrid
    return ArmSubsystem::m_pointHybrid;
}
Point const & ArmSubsystem::getLowPolePoint() { //  | Low Pole
    return ArmSubsystem::m_pointLowPole;
}
Point const & ArmSubsystem::getHighPolePoint() {//  | High Pole
    return ArmSubsystem::m_pointHighPole;
}
Point const & ArmSubsystem::getLowShelfPoint() {//  | Low Shelf
    return ArmSubsystem::m_pointLowShelf;
}
Point const & ArmSubsystem::getHighShelfPoint() {// | High Shelf
    return ArmSubsystem::m_pointHighShelf;
}

// Getting and setting arm angles:

double ArmSubsystem::getTurretAngle() {
    return m_turretAngleSensor.Get() * config.turret.sensorToRadians + config.turret.zeroOffset;
}

double ArmSubsystem::getShoulderAngle() {
    return m_shoulderAngleSensor.Get().value() * M_2_PI + config.shoulder.zeroOffset;
}

double ArmSubsystem::getElbowAngle() {
    return m_elbowAngleSensor.Get() * config.elbow.sensorToRadians + config.elbow.zeroOffset;
}

double ArmSubsystem::getWristRollAngle() {
    return m_wristRollAngleSensor.Get() * config.wrist.sensorToRadians + config.wrist.zeroOffset;
}

double ArmSubsystem::getGrip() {
    return m_gripSensor.Get() * config.grip.sensorToMeters + config.grip.zeroOffset;
}

Point ArmSubsystem::getWristPoint() {
    return calcWristPos(
        getTurretAngle(),
        getShoulderAngle(),
        getElbowAngle()
    );
}

Point const & ArmSubsystem::getSafetyPoint(Point pt) {
    if (isNearZero(pt.x)) {
        return ArmSubsystem::m_safetyPointCenter;
    } else if (0.0 > pt.x) {
        return ArmSubsystem::m_safetyPointGrid;
    } else {
        return ArmSubsystem::m_safetyPointIntake;
    }
}

void ArmSubsystem::setTurretAngle(double angle) {
    double da = angle - getTurretAngle();
    if (isNearZero(da)) {
        m_turretMotor.Set(0.0);
    } else {
        m_turretMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setShoulderAngle(double angle) {
    double da = angle - getShoulderAngle();
    if (isNearZero(da)) {
        m_lowJointMotor.Set(0.0);
    } else {
        m_lowJointMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setElbowAngle(double angle) {
    double da = angle - getElbowAngle();
    if (isNearZero(da)) {
        m_midJointMotor.Set(0.0);
    } else {
        m_midJointMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setWristRollAngle(double angle) {
    double da = angle - getWristRollAngle();
    if (isNearZero(da)) {
        m_gripperRotateMotor.Set(0.0);
    } else {
        m_gripperRotateMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setGrip(double grip) {
    double dx = grip - getGrip();
    if (isNearZero(dx)) {
        m_gripperGraspMotor.Set(0.0);
    } else {
        m_gripperGraspMotor.Set(std::copysign(0.05, dx));
    }
}

//          NOT IMPLEMENTED IN HARDWARE
// float ArmSubsystem::getWristPitchAngle() {
//     return 0; // TODO when we get reading components available
// }

/// LOCAL FREE
static double requireTomlDouble (
    std::shared_ptr<cpptoml::table> toml,
    std::string const & name
) {
    auto val = toml->get_qualified_as<double>(name);

    if (val) {
        return *val;
    } else {
        std::stringstream ss;
        ss << "Missing config option: arm." << name;
        throw ss;
    }
}

/// PRIVATE
void ArmSubsystem::loadConfig(std::shared_ptr<cpptoml::table> toml) {
    // Load zero offsets.

    config.turret.zeroOffset   = requireTomlDouble(toml, "turret.zeroOffsetRadians");
    config.shoulder.zeroOffset = requireTomlDouble(toml, "shoulder.zeroOffsetRadians");
    config.elbow.zeroOffset    = requireTomlDouble(toml, "elbow.zeroOffsetRadians");
    config.wrist.zeroOffset    = requireTomlDouble(toml, "wrist.zeroOffsetRadians");
    config.grip.zeroOffset     = requireTomlDouble(toml, "grip.zeroOffsetMeters");

    // Load conversion factors.

    config.turret.sensorToRadians = requireTomlDouble(toml, "turret.sensorToRadians");
    // No conversion factor for shoulder since sensor units are units::turns_t.
    config.elbow.sensorToRadians  = requireTomlDouble(toml, "elbow.sensorToRadians");
    config.wrist.sensorToRadians  = requireTomlDouble(toml, "wrist.sensorToRadians");
    config.grip.sensorToMeters    = requireTomlDouble(toml, "grip.sensorToMeters");

    // Load raw limits and convert to radians or meters.

    config.turret.limit.lo = requireTomlDouble(toml, "turret.limit.lo")
        * config.turret.sensorToRadians;
    config.turret.limit.hi = requireTomlDouble(toml, "turret.limit.hi")
        * config.turret.sensorToRadians;

    config.shoulder.limit.lo = requireTomlDouble(toml, "shoulder.limit.lo");
    config.shoulder.limit.hi = requireTomlDouble(toml, "shoulder.limit.hi");

    config.elbow.limit.lo = requireTomlDouble(toml, "elbow.limit.lo")
        * config.elbow.sensorToRadians;
    config.elbow.limit.hi = requireTomlDouble(toml, "elbow.limit.hi")
        * config.elbow.sensorToRadians;

    config.wrist.limit.lo = requireTomlDouble(toml, "wrist.limit.lo")
        * config.wrist.sensorToRadians;
    config.wrist.limit.hi = requireTomlDouble(toml, "wrist.limit.hi")
        * config.wrist.sensorToRadians;

    config.grip.limit.lo = requireTomlDouble(toml, "grip.limit.lo")
        * config.grip.sensorToMeters;
    config.grip.limit.hi = requireTomlDouble(toml, "grip.limit.hi")
        * config.grip.sensorToMeters;

    config.grip.setpoint.open = requireTomlDouble(toml, "grip.setpoint.open");
    config.grip.setpoint.close = requireTomlDouble(toml, "grip.setpoint.close");
}
