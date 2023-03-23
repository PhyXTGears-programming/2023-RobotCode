#include "Constants.h"
#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "util/math.h"

#include <numbers>
#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>

// PROTOTYPES
static double requireTomlDouble (std::shared_ptr<cpptoml::table> toml, std::string const & name);

// CONSTANTS
using Constants::Arm::k_ChassisXSize;
using Constants::Arm::k_ChassisYSize;
using Constants::Arm::k_ChassisZSize;

using Constants::Arm::k_TurretZOffset;

using Constants::Arm::k_PickupXSize;
using Constants::Arm::k_PickupYSize;

ArmSubsystem::ArmSubsystem(std::shared_ptr<cpptoml::table> toml) {
    loadConfig(toml);

    double kp, ki, kd, tolerance;

    kp = requireTomlDouble(toml, "shoulder.pid.p");
    ki = requireTomlDouble(toml, "shoulder.pid.i");
    kd = requireTomlDouble(toml, "shoulder.pid.d");
    tolerance = requireTomlDouble(toml, "shoulder.pid.tolerance");

    c_shoulderPid = new frc2::PIDController(kp, ki, kd);
    c_shoulderPid->SetTolerance(tolerance);
    c_shoulderPid->SetIntegratorRange(-0.01, 0.01);

    c_shoulderPid->SetSetpoint(getShoulderAngle());

    c_gripperRotateMotor.SetInverted(true); // CW is (+) speed
    c_gripperGraspMotor.SetInverted(true); // Grip open is (+) speed

    c_turretMotor.SetSmartCurrentLimit(20.0);
    c_lowJointMotor.SetSmartCurrentLimit(25.0);
    c_midJointMotor.SetSmartCurrentLimit(20.0);
    c_gripperRotateMotor.SetSmartCurrentLimit(5.0);
    c_gripperGraspMotor.SetSmartCurrentLimit(30.0);

    c_turretMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    c_lowJointMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    c_midJointMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    c_gripperRotateMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    c_gripperGraspMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    initialiseBoundary();
}

void ArmSubsystem::Periodic() {
    m_computedGripPoint = calcGripPos(getTurretAngle(), getShoulderAngle(), getElbowAngle());

    // Move shoulder or hold position.
    // Must update motor output here to hold shoulder position because arm backdrives due to
    // gravity.
    if (nullptr != c_shoulderPid) {
        double output = c_shoulderPid->Calculate(getShoulderAngle());
        // Reverse motor direction.
        output = -output;
        output = std::clamp(output, -0.2, 0.2);
        c_lowJointMotor.Set(output);
    }
}

void ArmSubsystem::initialiseBoundary() {
    // Geometric Conventions:
    //  x-axis 0 :: center of arm turret.
    //  y-axis 0 :: center of arm turret.
    //  z-axis 0 :: floor.
    //
    //  x+ :: robot right side (with pickup gap in bumpers).
    //  y+ :: robot front (furthest side from turret).
    //  z+ :: up.

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

    std::shared_ptr<Boundary> cannotReachNoGoBounds = std::make_shared<NotBoundary>(
        std::make_unique<SphereBoundary>(
            Point { 0.0, 0.0, k_ChassisZSize }, /* center of turret */
            (Constants::Arm::k_forearmLenMeters
                + Constants::Arm::k_bicepLenMeters
            ) * 0.95 /* 95% of max length of arm */
        )
    );

    std::shared_ptr<Boundary> turretNoGoZone = std::make_shared<CylinderBoundary>(
        Point { 0.0, 0.0, 0.0 },
        0.08, /* radius meters */
        0.0, 1.0
    );

    std::shared_ptr<Boundary> floorNoGoZone = std::make_shared<BoxBoundary>(
        -5.0,     5.0,
        -5.0,     5.0,
        -5.0,     0.0254   /* keep gripper 1 inch from floor. 1 inch = 0.0254 meters */
    );

    std::shared_ptr<ComposeBoundary> defNoGoZone = std::make_shared<ComposeBoundary>(std::vector{
        std::move(cannotReachNoGoBounds),
        std::move(chassisNoGoBounds),
        std::move(turretNoGoZone),
        std::move(floorNoGoZone),
    });

    c_noGoZone = std::move(defNoGoZone);
}

Point ArmSubsystem::calcElbowPos(double turretAng, double shoulderAng) {
    // Origin z is floor level.  Add turret z offset to calculated point.
    Point pt = Point(
        Constants::Arm::k_bicepLenMeters * std::cos(shoulderAng) * std::sin(turretAng),
        Constants::Arm::k_bicepLenMeters * std::cos(shoulderAng) * std::cos(turretAng),
        Constants::Arm::k_bicepLenMeters * std::sin(shoulderAng)
    ) + Vector(0.0, 0.0, k_TurretZOffset);

    return pt;
}

Point ArmSubsystem::calcGripPos(
    double turretAng,
    double shoulderAng,
    double elbowAng
) {
    Point elbowPos = calcElbowPos(turretAng, shoulderAng);
    Point pt(
        Constants::Arm::k_forearmLenMeters * std::cos(shoulderAng + elbowAng) * std::sin(turretAng),
        Constants::Arm::k_forearmLenMeters * std::cos(shoulderAng + elbowAng) * std::cos(turretAng),
        Constants::Arm::k_forearmLenMeters * std::sin(shoulderAng + elbowAng)
    );

    return Point(elbowPos.x - pt.x, elbowPos.y - pt.y, elbowPos.z - pt.z);
}

ArmPose ArmSubsystem::calcIKJointPoses(Point const & pt) {
    // See arm.h for diagram of robot arm angle reference conventions.
    // desmos 2d IK solver demo: https://www.desmos.com/calculator/p3uouu2un2

    // IK solver assumes base of turret is origin.  Adjust target point from
    // robot origin to turret origin.
    Point turretPt = pt - Vector(0.0, 0.0, k_TurretZOffset);

    double targetLen =  std::sqrt(
        std::pow(turretPt.x, 2)
        + std::pow(turretPt.y, 2)
        + std::pow(turretPt.z, 2)
    ); // Line from shoulder to target

    // Constrain target point.
    double constrainLen = std::min(
        Constants::Arm::k_forearmLenMeters + Constants::Arm::k_bicepLenMeters * 0.95,   /* Safety margin. Limit length to 95% of max. */
        targetLen
    );

    Point cp = Point(0.0, 0.0, 0.0) + Vector(turretPt.x, turretPt.y, turretPt.z).unit() * constrainLen;

    double c3 = atan2(cp.z, std::sqrt(cp.x * cp.x + cp.y * cp.y));

    double c1 = std::acos(
        (Constants::Arm::k_bicepLenMeters * Constants::Arm::k_bicepLenMeters
        - Constants::Arm::k_forearmLenMeters * Constants::Arm::k_forearmLenMeters
        + constrainLen * constrainLen)
        / (2.0 * Constants::Arm::k_bicepLenMeters * constrainLen)
    );

    // Solve IK:
    double shoulderAngle = c1 + c3;

    double elbowAngle =
        std::numbers::pi
        - std::acos(
            (
                Constants::Arm::k_forearmLenMeters * Constants::Arm::k_forearmLenMeters
                - Constants::Arm::k_bicepLenMeters * Constants::Arm::k_bicepLenMeters
                + constrainLen * constrainLen
            )
            / (2.0 * Constants::Arm::k_forearmLenMeters * constrainLen)
        )
        - c1;

    double turretAngle = std::atan2(cp.x, cp.y);

    return ArmPose(turretAngle, shoulderAngle, elbowAngle);

}

bool ArmSubsystem::isPointSafe(Point const & point) {
    if (nullptr == c_noGoZone) {
        return false;
    } else {
        return c_noGoZone->isOutside(point);
    }
}

bool ArmSubsystem::isNearPoint(Point const & point) {
    Point currentPosition = getGripPoint();

    return point.isNear(currentPosition);
}

MotionPath ArmSubsystem::getPathTo(Point const & current, Point const & target) {
    std::vector<Point> path;

    SafetyZone startZone  = getSafetyZone(current);
    SafetyZone targetZone = getSafetyZone(target);

    switch (startZone) {
    case ArmSubsystem::SafetyZone::LEFT:
        switch (targetZone) {
        case ArmSubsystem::SafetyZone::LEFT:
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::MIDDLE:
            path.push_back(m_safetyPointIntake);
            path.push_back(m_safetyPointCenter);
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::RIGHT:
            path.push_back(m_safetyPointIntake);
            path.push_back(m_safetyPointCenter);
            path.push_back(m_safetyPointGrid);
            path.push_back(target);
            break;
        }
        break;

    case ArmSubsystem::SafetyZone::MIDDLE:
        switch (targetZone) {
        case ArmSubsystem::SafetyZone::LEFT:
            path.push_back(m_safetyPointCenter);
            path.push_back(m_safetyPointIntake);
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::MIDDLE:
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::RIGHT:
            path.push_back(m_safetyPointCenter);
            path.push_back(m_safetyPointGrid);
            path.push_back(target);
            break;
        }
        break;

    case ArmSubsystem::SafetyZone::RIGHT:
        switch (targetZone) {
        case ArmSubsystem::SafetyZone::LEFT:
            path.push_back(m_safetyPointGrid);
            path.push_back(m_safetyPointCenter);
            path.push_back(m_safetyPointIntake);
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::MIDDLE:
            path.push_back(m_safetyPointGrid);
            path.push_back(m_safetyPointCenter);
            path.push_back(target);
            break;

        case ArmSubsystem::SafetyZone::RIGHT:
            path.push_back(target);
            break;
        }
        break;
    }

    return MotionPath(path);
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
    return c_turretAngleSensor.Get() * config.turret.sensorToRadians + config.turret.zeroOffset;
}

double ArmSubsystem::getShoulderAngle() {
    return c_shoulderAngleSensor.Get().value() * 2.0 * M_PI + config.shoulder.zeroOffset;
}

double ArmSubsystem::getElbowAngle() {
    return c_elbowAngleSensor.Get() * config.elbow.sensorToRadians + config.elbow.zeroOffset;
}

double ArmSubsystem::getWristRollAngle() {
    return c_wristRollAngleSensor.Get() * config.wrist.sensorToRadians + config.wrist.zeroOffset;
}

double ArmSubsystem::getGrip() {
    return c_gripSensor.Get() * config.grip.sensorToMeters + config.grip.zeroOffset;
}

Point ArmSubsystem::getGripPoint() {
    return m_computedGripPoint;
}

ArmSubsystem::SafetyZone ArmSubsystem::getSafetyZone(Point const & pt) {
    if (-k_ChassisXSize / 2.0 > pt.x) {
        return SafetyZone::LEFT;
    } else if (k_ChassisXSize / 2.0 < pt.x) {
        return SafetyZone::RIGHT;
    } else {
        return SafetyZone::MIDDLE;
    }
}

void ArmSubsystem::setTurretAngle(double angle) {
    angle = std::clamp(angle, config.turret.limit.lo, config.turret.limit.hi);
    _setTurretAngle(angle);
}

/// @brief DO NOT USE without clamping angle between limits.
/// @param angle 
void ArmSubsystem::_setTurretAngle(double angle) {
    if (std::isnan(angle)) {
        return;
    }

    double da = angle - getTurretAngle();
    if (isNearZero(da, 0.02)) {
        c_turretMotor.Set(0.0);
    } else {
        c_turretMotor.Set(std::clamp(da, -0.1, 0.1));
    }
}

void ArmSubsystem::setShoulderAngle(double angle) {
    angle = std::clamp(angle, config.shoulder.limit.lo, config.shoulder.limit.hi);
    _setShoulderAngle(angle);
}

/// @brief DO NOT USE without clamping angle between limits.
/// @param angle 
void ArmSubsystem::_setShoulderAngle(double angle) {
    if (std::isnan(angle)) {
        return;
    }

    // Use PID to hold arm in place once target angle is reached.
    c_shoulderPid->SetSetpoint(angle);
}

void ArmSubsystem::setElbowAngle(double angle) {
    angle = std::clamp(angle, config.elbow.limit.lo, config.elbow.limit.hi);
    _setElbowAngle(angle);
}

/// @brief DO NOT USE without clamping angle between limits.
/// @param angle 
void ArmSubsystem::_setElbowAngle(double angle) {
    if (std::isnan(angle)) {
        return;
    }

    double da = angle - getElbowAngle();
    if (isNearZero(da, 0.02)) {
        c_midJointMotor.Set(0.0);
    } else {
        c_midJointMotor.Set(std::clamp(da, -0.05, 0.05) + std::copysign(0.1, da));
    }
}

void ArmSubsystem::setWristRollAngle(double angle) {
    if (std::isnan(angle)) {
        return;
    }

    angle = std::clamp(angle, config.wrist.limit.lo, config.wrist.limit.hi);

    double da = angle - getWristRollAngle();
    if (isNearZero(da, 0.01)) {
        c_gripperRotateMotor.Set(0.0);
    } else {
        c_gripperRotateMotor.Set(std::clamp(
            da + std::copysign(0.05, da),
            -0.2, 0.2
        ));
    }
}

void ArmSubsystem::setGrip(double grip) {
    if (std::isnan(grip)) {
        return;
    }

    grip = std::clamp(grip, config.grip.limit.lo, config.grip.limit.hi);

    double dx = grip - getGrip();
    if (isNearZero(dx, 0.015 /* meters */)) {
        c_gripperGraspMotor.Set(0.0);
    } else {
        c_gripperGraspMotor.Set(std::clamp(
            dx + std::copysign(0.15, dx),
            -0.2, 0.2
        ));
    }
}

void ArmSubsystem::setTurretSpeed(double speed) {
    c_turretMotor.Set(speed);
}

void ArmSubsystem::setShoulderSpeed(double speed) {
    c_lowJointMotor.Set(speed);
}

void ArmSubsystem::setElbowSpeed(double speed) {
    c_midJointMotor.Set(speed);
}

void ArmSubsystem::setWristRollSpeed(double speed) {
    c_gripperRotateMotor.Set(speed);
}

void ArmSubsystem::setGripSpeed(double speed) {
    c_gripperGraspMotor.Set(speed);
}

std::optional<Point> ArmSubsystem::moveToPoint(Point const & target) {
    // FIXME: Disabled check for safe point until testing resolves issue with all points unsafe.
    // Lean on angle and speed limits for safety in the meantime.
    frc::SmartDashboard::PutBoolean("Is target point safe?", isPointSafe(target));
    //if (!isPointSafe(target)) {
    //    return std::nullopt;
    //}

    ArmPose pose = calcIKJointPoses(target);

    pose.turretAngle = std::clamp(pose.turretAngle, config.turret.limit.lo, config.turret.limit.hi);
    pose.shoulderAngle = std::clamp(pose.shoulderAngle, config.shoulder.limit.lo, config.shoulder.limit.hi);
    pose.elbowAngle = std::clamp(pose.elbowAngle, config.elbow.limit.lo, config.elbow.limit.hi);

    // If any angle is Not a Number (NaN), target point is unsafe.
    if (std::isnan(pose.turretAngle)
        || std::isnan(pose.shoulderAngle)
        || std::isnan(pose.elbowAngle))
    {
        return std::nullopt;
    }

    _setTurretAngle(pose.turretAngle);
    _setShoulderAngle(pose.shoulderAngle);
    _setElbowAngle(pose.elbowAngle);

    Point checkTarget = calcGripPos(pose.turretAngle, pose.shoulderAngle, pose.elbowAngle);

    return checkTarget;
}

void ArmSubsystem::stopArm() {
    moveToPoint(getGripPoint());
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

    config.turret.limit.lo = DEG_2_RAD(requireTomlDouble(toml, "turret.limit.lo"));
    config.turret.limit.hi = DEG_2_RAD(requireTomlDouble(toml, "turret.limit.hi"));

    config.shoulder.limit.lo = DEG_2_RAD(requireTomlDouble(toml, "shoulder.limit.lo"));
    config.shoulder.limit.hi = DEG_2_RAD(requireTomlDouble(toml, "shoulder.limit.hi"));

    config.elbow.limit.lo = DEG_2_RAD(requireTomlDouble(toml, "elbow.limit.lo"));
    config.elbow.limit.hi = DEG_2_RAD(requireTomlDouble(toml, "elbow.limit.hi"));

    config.wrist.limit.lo = DEG_2_RAD(requireTomlDouble(toml, "wrist.limit.lo"));
    config.wrist.limit.hi = DEG_2_RAD(requireTomlDouble(toml, "wrist.limit.hi"));

    config.grip.limit.lo = requireTomlDouble(toml, "grip.limit.lo");
    config.grip.limit.hi = requireTomlDouble(toml, "grip.limit.hi");

    config.grip.setpoint.open = requireTomlDouble(toml, "grip.setpoint.open");
    config.grip.setpoint.close = requireTomlDouble(toml, "grip.setpoint.close");
}
