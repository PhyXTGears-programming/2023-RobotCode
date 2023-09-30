#include "subsystems/arm/arm.h"
#include "Constants.h"
#include "Mandatory.h"
#include "util/geom.h"
#include "util/math.h"

#include <cmath>
#include <numbers>

#include <frc/smartdashboard/SmartDashboard.h>

// PROTOTYPES
static double requireTomlDouble(std::shared_ptr<cpptoml::table> toml, const std::string & name);

ArmSubsystem::ArmSubsystem(std::shared_ptr<cpptoml::table> toml) {
    loadConfig(toml);

    double kp, ki, kd, tolerance;

    kp        = requireTomlDouble(toml, "shoulder.pid.p");
    ki        = requireTomlDouble(toml, "shoulder.pid.i");
    kd        = requireTomlDouble(toml, "shoulder.pid.d");
    tolerance = requireTomlDouble(toml, "shoulder.pid.tolerance");

    c_shoulderPid = new frc2::PIDController(kp, ki, kd);
    c_shoulderPid->SetTolerance(tolerance);
    c_shoulderPid->SetIntegratorRange(-0.01, 0.01);

    resetShoulderAngle();

    c_lowJointMotor.SetInverted(true);

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

    m_elbowSensorMeasurements[0] = c_elbowAngleSensor.Get();
    m_elbowSensorMeasurements[1] = m_elbowSensorMeasurements[0];
    m_elbowSensorMeasurements[2] = m_elbowSensorMeasurements[0];

    m_elbowSensorAverage = m_elbowSensorMeasurements[0];

    AddChild("Camera Servo", &c_cameraServo);
}

void ArmSubsystem::Periodic() {
    static Vector i{1.0, 0.0};

    _updateElbowAverage();

    m_computedGripPoint = calcGripPos(getShoulderAngle(), getElbowAngle());

    // Move shoulder or hold position.
    // Must update motor output here to hold shoulder position because arm backdrives due to
    // gravity.
    if (nullptr != c_shoulderPid) {
        double output = c_shoulderPid->Calculate(getShoulderAngle());

        // Adjust feed forward gravity compensation depending on arm extension.
        Vector gripVec = Vector{ m_computedGripPoint.x, m_computedGripPoint.y }.unit();
        double gravityCompensationFactor = gripVec.unit().dot(i) / std::max(gripVec.len(), 1.0);

        if (!isNearZero(output, 0.006)) {
            if (output < 0.0) {
                output -= 0.005;
            } else {
                output += 0.06 * gravityCompensationFactor;
            }
        }

        output = std::clamp(output, -0.15, 0.20);

        frc::SmartDashboard::PutNumber("shoulder pid output", output);
        frc::SmartDashboard::PutNumber("shoulder pid error", c_shoulderPid->GetPositionError());

        c_lowJointMotor.Set(output);
    }

    {
        // clang-format off
        // Reset camera servo position to default.  Will eventually track grip position, if we're lucky.
        // c_cameraServo.SetAngle(50.0);
        // 29 -> 140
        // 110 -> 50
        // clang-format on

        // Try to follow gripper mechanism based on elbow angle.
        double elbowAngle = RAD_2_DEG(getElbowAngle());
        double servoAngle = std::clamp(180.0 + elbowAngle * (90 / -71.0) + 10.0, 0.0, 180.0);
        c_cameraServo.SetAngle(servoAngle);
    }

    frc::SmartDashboard::PutNumber("Turret Angle (deg)", RAD_2_DEG(getTurretAngle()));
    frc::SmartDashboard::PutNumber("Shoulder Angle (deg)", RAD_2_DEG(getShoulderAngle()));
    frc::SmartDashboard::PutNumber("Elbow Angle (deg)", RAD_2_DEG(getElbowAngle()));
    frc::SmartDashboard::PutNumber("Wrist Roll Angle (deg)", RAD_2_DEG(getWristRollAngle()));
    frc::SmartDashboard::PutNumber("Grip Distance (meters)", getGrip());

#ifdef DEBUG_MODE
    // Raw values
    frc::SmartDashboard::PutNumber("Raw: Turret Angle (V)", c_turretAngleSensor.Get());
    frc::SmartDashboard::PutNumber(
        "Raw: Shoulder Angle (deg)",
        c_shoulderAngleSensor.Get().value() * 360.0);
    frc::SmartDashboard::PutNumber("Raw: Elbow Angle (V)", c_elbowAngleSensor.Get());
    frc::SmartDashboard::PutNumber("Raw: Wrist Roll Angle (V)", c_wristRollAngleSensor.Get());
    frc::SmartDashboard::PutNumber("Raw: Grip Distance (V)", c_gripSensor.Get());
#endif

    // Report calculated gripper position.
    Point gripPos = getGripPoint();
    {
        static double gripCoords[3];
        gripCoords[0] = gripPos.x;
        gripCoords[1] = gripPos.y;
        frc::SmartDashboard::PutNumberArray("Grip Pos (m)", gripCoords);
    }

    // Report calculated arm pose angles.  What the robot thinks the angles
    // should be to reach gripper position.

    ArmPose pose = calcIKJointPoses(gripPos);
    frc::SmartDashboard::PutNumber("IK Shoulder Angle (deg)", RAD_2_DEG(pose.shoulderAngle));
    frc::SmartDashboard::PutNumber("IK Elbow Angle (deg)", RAD_2_DEG(pose.elbowAngle));
}

Point ArmSubsystem::calcElbowPos(double shoulderAng) {
    // Origin z is floor level.  Add turret z offset to calculated point.
    Point pt = Point(
        Constants::Arm::k_bicepLenMeters * std::cos(shoulderAng),
        Constants::Arm::k_bicepLenMeters * std::sin(shoulderAng));

    return pt;
}

Point ArmSubsystem::calcGripPos(double shoulderAng, double elbowAng) {
    Point elbowPos = calcElbowPos(shoulderAng);
    Point pt(
        Constants::Arm::k_forearmLenMeters * std::cos(shoulderAng + elbowAng),
        Constants::Arm::k_forearmLenMeters * std::sin(shoulderAng + elbowAng));

    return Point(elbowPos.x - pt.x, elbowPos.y - pt.y);
}

ArmPose ArmSubsystem::calcIKJointPoses(const Point & pt) {
    // See arm.h for diagram of robot arm angle reference conventions.
    // desmos 2d IK solver demo: https://www.desmos.com/calculator/p3uouu2un2

    // IK solver assumes base of turret is origin.  Adjust target point from
    // robot origin to turret origin.
    Point turretPt = pt;

    double targetLen = std::sqrt(
        std::pow(turretPt.x, 2) + std::pow(turretPt.y, 2)); // Line from shoulder to target

    // Constrain target point.
    double constrainLen = std::min(
        Constants::Arm::k_forearmLenMeters
            + Constants::Arm::k_bicepLenMeters
                  * 0.95, /* Safety margin. Limit length to 95% of max. */
        targetLen);

    Point cp = Point(0.0, 0.0) + Vector(turretPt.x, turretPt.y).unit() * constrainLen;

    double c3 = atan2(cp.y, cp.x);

    double c1 = std::acos(
        (Constants::Arm::k_bicepLenMeters * Constants::Arm::k_bicepLenMeters
         - Constants::Arm::k_forearmLenMeters * Constants::Arm::k_forearmLenMeters
         + constrainLen * constrainLen)
        / (2.0 * Constants::Arm::k_bicepLenMeters * constrainLen));

    // Solve IK:
    double shoulderAngle = c1 + c3;

    double elbowAngle = std::numbers::pi
                        - std::acos(
                            (Constants::Arm::k_forearmLenMeters * Constants::Arm::k_forearmLenMeters
                             - Constants::Arm::k_bicepLenMeters * Constants::Arm::k_bicepLenMeters
                             + constrainLen * constrainLen)
                            / (2.0 * Constants::Arm::k_forearmLenMeters * constrainLen))
                        - c1;

    return ArmPose(shoulderAngle, elbowAngle);
}

bool ArmSubsystem::isNearPoint(const Point & point) {
    Point currentPosition = getGripPoint();

    return point.isNear(currentPosition);
}

// Getting and setting arm angles:

double ArmSubsystem::getTurretAngle() {
    return c_turretAngleSensor.Get() * config.turret.sensorToRadians + config.turret.zeroOffset;
}

double ArmSubsystem::getShoulderAngle() {
    return c_shoulderAngleSensor.Get().value() * 2.0 * M_PI + config.shoulder.zeroOffset;
}

double ArmSubsystem::getElbowAngle() {
    // Updated via Periodic.  Calculated in _updateElbowAverage.
    return m_elbowSensorAverage;
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

void ArmSubsystem::setTurretAngle(double angle) {
    // WARNING. IK for turret is removed.  Use setTurretSpeed() instead.
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
        da += std::copysign(0.05, da);
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
    if (isNearZero(da, 0.01)) {
        c_midJointMotor.Set(0.0);
    } else {
        if (da > 0.0) {
            // Help arm move up.
            da += 0.15;
        } else {
            da += -0.1;
        }
        c_midJointMotor.Set(std::clamp(da, -0.13, 0.20));
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
        da += std::copysign(0.05, da);
        c_gripperRotateMotor.Set(std::clamp(da, -0.2, 0.2));
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
        dx += std::copysign(0.15, dx);
        c_gripperGraspMotor.Set(std::clamp(dx, -1.0, 1.0));
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

std::optional<Point> ArmSubsystem::moveToPoint(const Point & target) {
    ArmPose pose = calcIKJointPoses(target);

    pose.shoulderAngle =
        std::clamp(pose.shoulderAngle, config.shoulder.limit.lo, config.shoulder.limit.hi);
    pose.elbowAngle = std::clamp(pose.elbowAngle, config.elbow.limit.lo, config.elbow.limit.hi);

    // If any angle is Not a Number (NaN), target point is unsafe.
    if (std::isnan(pose.shoulderAngle) || std::isnan(pose.elbowAngle)) {
        return std::nullopt;
    }

    _setShoulderAngle(pose.shoulderAngle);
    _setElbowAngle(pose.elbowAngle);

    Point checkTarget = calcGripPos(pose.shoulderAngle, pose.elbowAngle);

    return checkTarget;
}

void ArmSubsystem::stopArm() {
    moveToPoint(getGripPoint());
}

void ArmSubsystem::resetShoulderAngle() {
    c_shoulderPid->SetSetpoint(getShoulderAngle());
}

//          NOT IMPLEMENTED IN HARDWARE
// float ArmSubsystem::getWristPitchAngle() {
//     return 0; // TODO when we get reading components available
// }

/// LOCAL FREE
static double requireTomlDouble(std::shared_ptr<cpptoml::table> toml, const std::string & name) {
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
    config.elbow.sensorToRadians = requireTomlDouble(toml, "elbow.sensorToRadians");
    config.wrist.sensorToRadians = requireTomlDouble(toml, "wrist.sensorToRadians");
    config.grip.sensorToMeters   = requireTomlDouble(toml, "grip.sensorToMeters");

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

    config.grip.setpoint.open  = requireTomlDouble(toml, "grip.setpoint.open");
    config.grip.setpoint.close = requireTomlDouble(toml, "grip.setpoint.close");
}

void ArmSubsystem::_updateElbowAverage() {
    m_elbowSensorMeasurements[0] = m_elbowSensorMeasurements[1];
    m_elbowSensorMeasurements[1] = m_elbowSensorMeasurements[2];
    m_elbowSensorMeasurements[2] =
        c_elbowAngleSensor.Get() * config.elbow.sensorToRadians + config.elbow.zeroOffset;

    m_elbowSensorAverage = 0.15 * m_elbowSensorMeasurements[0] + 0.25 * m_elbowSensorMeasurements[1] + 0.6 * m_elbowSensorMeasurements[2];
}
