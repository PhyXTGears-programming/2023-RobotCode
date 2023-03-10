#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "util/math.h"

#include <numbers>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <frc/smartdashboard/SmartDashboard.h>

// PROTOTYPES
static double requireTomlDouble (std::shared_ptr<cpptoml::table> toml, std::string const & name);

// CONSTANTS
const double k_ChassisXSize = 0.775; // meters
const double k_ChassisYSize = 0.826; // meters
const double k_ChassisZSize = 0.229; // meters

const double k_TurretZOffset = 0.210; // meters

const double k_PickupXSize = 0.076;  // meters
const double k_PickupYSize = 0.340;  // meters
const double k_PickupZSize = 1.000;  // meters

ArmSubsystem::ArmSubsystem(std::shared_ptr<cpptoml::table> toml) {
    loadConfig(toml);

    double kp, ki, kd, tolerance;

    kp = requireTomlDouble(toml, "shoulder.pid.p");
    ki = requireTomlDouble(toml, "shoulder.pid.i");
    kd = requireTomlDouble(toml, "shoulder.pid.d");
    tolerance = requireTomlDouble(toml, "shoulder.pid.tolerance");

    m_shoulderPid = new frc2::PIDController(kp, ki, kd);
    m_shoulderPid->SetTolerance(tolerance);
    m_shoulderPid->SetIntegratorRange(-0.01, 0.01);

    m_shoulderPid->SetSetpoint(getShoulderAngle());
}

void ArmSubsystem::Periodic() {
    // Move shoulder or hold position.
    if (nullptr != m_shoulderPid) {
        double output = m_shoulderPid->Calculate(getShoulderAngle());
        m_turretMotor.Set(output);
    }

    frc::SmartDashboard::PutNumber("Turret Angle (deg)",     RAD_2_DEG(getTurretAngle()));
    frc::SmartDashboard::PutNumber("Shoulder Angle (deg)",   RAD_2_DEG(getShoulderAngle()));
    frc::SmartDashboard::PutNumber("Elbow Angle (deg)",      RAD_2_DEG(getElbowAngle()));
    frc::SmartDashboard::PutNumber("Wrist Roll Angle (deg)", RAD_2_DEG(getWristRollAngle()));
    frc::SmartDashboard::PutNumber("Grip Distance (meters)", getGrip());

    // Raw values
    frc::SmartDashboard::PutNumber("Raw: Turret Angle (V)",     m_turretAngleSensor.Get());
    frc::SmartDashboard::PutNumber("Raw: Shoulder Angle (deg)", m_shoulderAngleSensor.Get().value() * 360.0);
    frc::SmartDashboard::PutNumber("Raw: Elbow Angle (V)",      m_elbowAngleSensor.Get());
    frc::SmartDashboard::PutNumber("Raw: Wrist Roll Angle (V)", m_wristRollAngleSensor.Get());
    frc::SmartDashboard::PutNumber("Raw: Grip Distance (V)",    m_gripSensor.Get());

    // Report calculated gripper position.

    Point gripPos = getGripPoint();
    std::stringstream gripPosStr;

    gripPosStr << std::fixed << std::setprecision(4)
        << "(" << gripPos.x
        << ", " << gripPos.y
        << ", " << gripPos.z
        << ")";
    frc::SmartDashboard::PutString("Grip Pos (m)", gripPosStr.str());

    // Report calculated arm pose angles.  What the robot thinks the angles
    // should be to reach gripper position.

    ArmPose pose = calcIKJointPoses(gripPos);
    frc::SmartDashboard::PutNumber("IK Turret Angle (deg)",   RAD_2_DEG(pose.turretAngle));
    frc::SmartDashboard::PutNumber("IK Shoulder Angle (deg)", RAD_2_DEG(pose.shoulderAngle));
    frc::SmartDashboard::PutNumber("IK Elbow Angle (deg)",    RAD_2_DEG(pose.elbowAngle));
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
         0.0254, -5.0   /* keep gripper 1 inch from floor */
    );

    std::shared_ptr<ComposeBoundary> defNoGoZone = std::make_shared<ComposeBoundary>(std::vector{
        std::move(cannotReachNoGoBounds),
        std::move(chassisNoGoBounds),
        std::move(turretNoGoZone),
        std::move(floorNoGoZone),
    });

    m_noGoZone = std::move(defNoGoZone);
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
    if (nullptr == m_noGoZone) {
        return false;
    } else {
        return m_noGoZone->isOutside(point);
    }
}

bool ArmSubsystem::isNearPoint(Point const & point) {
    Point currentPosition = getGripPoint();

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
    return m_shoulderAngleSensor.Get().value() * 2.0 * M_PI + config.shoulder.zeroOffset;
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

Point ArmSubsystem::getGripPoint() {
    return calcGripPos(
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
    angle = std::clamp(angle, config.turret.limit.lo, config.turret.limit.hi);

    double da = angle - getTurretAngle();
    if (isNearZero(da)) {
        m_turretMotor.Set(0.0);
    } else {
        m_turretMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setShoulderAngle(double angle) {
    angle = std::clamp(angle, config.shoulder.limit.lo, config.shoulder.limit.hi);

    /*
    double da = angle - getShoulderAngle();
    if (isNearZero(da)) {
        m_lowJointMotor.Set(0.0);
    } else {
        m_lowJointMotor.Set(std::copysign(0.05, da));
    }
    */

    // Use PID to hold arm in place once target angle is reached.
    m_shoulderPid->SetSetpoint(angle);
}

void ArmSubsystem::setElbowAngle(double angle) {
    angle = std::clamp(angle, config.elbow.limit.lo, config.elbow.limit.hi);

    double da = angle - getElbowAngle();
    if (isNearZero(da)) {
        m_midJointMotor.Set(0.0);
    } else {
        m_midJointMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setWristRollAngle(double angle) {
    angle = std::clamp(angle, config.wrist.limit.lo, config.wrist.limit.hi);

    double da = angle - getWristRollAngle();
    if (isNearZero(da)) {
        m_gripperRotateMotor.Set(0.0);
    } else {
        m_gripperRotateMotor.Set(std::copysign(0.05, da));
    }
}

void ArmSubsystem::setGrip(double grip) {
    grip = std::clamp(grip, config.grip.limit.lo, config.grip.limit.hi);

    double dx = grip - getGrip();
    if (isNearZero(dx)) {
        m_gripperGraspMotor.Set(0.0);
    } else {
        m_gripperGraspMotor.Set(std::copysign(0.05, dx));
    }
}

bool ArmSubsystem::moveToPoint(Point const & target) {
    if (!isPointSafe(target)) {
        return false;
    }

    ArmPose pose = calcIKJointPoses(target);

    setTurretAngle(pose.turretAngle);
    setShoulderAngle(pose.shoulderAngle);
    setElbowAngle(pose.elbowAngle);

    return true;
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
