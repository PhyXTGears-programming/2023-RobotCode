#pragma once

#include "Mandatory.h"
#include "subsystems/arm/armPose.h"
#include "subsystems/arm/motionPath.h"
#include "util/geom.h"

#include "external/cpptoml.h"

#include <cmath>
#include <memory>
#include <optional>

#include <frc/AnalogPotentiometer.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Servo.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

using namespace Interfaces::Arm;

class ArmSubsystem : public frc2::SubsystemBase {
public:
    enum class SafetyZone {
        LEFT,
        MIDDLE,
        RIGHT,
    };

    ArmSubsystem(std::shared_ptr<cpptoml::table> toml);

    void Periodic() override;

    void initialiseBoundary();

    // Calulating Functions:
    Point calcElbowPos(double angShoulder);

    Point calcGripPos(double shoulderAng, double elbowAng);

    ArmPose calcIKJointPoses(const Point & pt);

    bool isPointSafe(const Point & point);
    bool isNearPoint(const Point & point);

    MotionPath getPathTo(const Point & current, const Point & target);

    // Reading Functions:
    double getTurretAngle();
    double getShoulderAngle();
    double getElbowAngle();
    double getWristRollAngle();
    double getGrip();

    // Getting Points:
    Point getGripPoint();

    SafetyZone getSafetyZone(const Point & pt);

    void setTurretAngle(double angle);
    void setShoulderAngle(double angle);
    void setElbowAngle(double angle);
    void setWristRollAngle(double angle);
    void setGrip(double grip); // hehe grippy grabby hand

    void setTurretSpeed(double speed);
    void setShoulderSpeed(double speed);
    void setElbowSpeed(double speed);
    void setWristRollSpeed(double speed);
    void setGripSpeed(double speed);

    /**
     * Given a target position, compute necessary joint angles, and move joints
     * of arm toward target.  Will NOT stop on its own.  Must call this method
     * repeatedly to stop on target.
     *
     * @return false if point is within the no-go zone, true if point is safe.
     */
    std::optional<Point> moveToPoint(const Point & target);

    void stopArm();

    void resetShoulderAngle();

private:
    void loadConfig(std::shared_ptr<cpptoml::table> toml);

    void _setTurretAngle(double angle);
    void _setShoulderAngle(double angle);
    void _setElbowAngle(double angle);

    void _updateElbowAverage();

    // Arm Diagram:
    // 1: Turret, 2: Shoulder, 3: Elbow, 4: Wrist
    //
    //           (3)---(4){
    //          /
    //        [2]
    //        (1)
    //  [=====1720=====]
    //     O        O
    // **Not to scale
    // Shoulder = Low Joint, Elbow = Mid Joint, Wrist = Gripper Rotate/Roll
    //
    // Turret angle conventions/reference
    //               0 deg
    //          ┌──────────┐
    //          │          │
    //          │          │
    //          │          │
    //  -90 deg │    o     │ 90 deg
    //          │          │
    //          └──────────┘

    rev::CANSparkMax c_turretMotor{ k_TurretMotor,
                                    rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANSparkMax c_lowJointMotor{ k_LowJointMotor,
                                      rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANSparkMax c_midJointMotor{ k_MidJointMotor,
                                      rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANSparkMax c_gripperRotateMotor{ k_GripperRotateMotor,
                                           rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::CANSparkMax c_gripperGraspMotor{ k_GripperGraspMotor,
                                          rev::CANSparkMaxLowLevel::MotorType::kBrushless };

    frc::Servo c_cameraServo{ k_CameraServo };

    frc::AnalogPotentiometer c_turretAngleSensor{ k_TurretSensor, 1.0, 0.0 };
    frc::AnalogPotentiometer c_elbowAngleSensor{ k_ElbowSensor, 1.0, 0.0 };
    frc::AnalogPotentiometer c_wristRollAngleSensor{ k_WristRollSensor, 1.0, 0.0 };
    frc::AnalogPotentiometer c_gripSensor{ k_GripSensor, 1.0, 0.0 };
    frc::DutyCycleEncoder c_shoulderAngleSensor{ k_ShoulderSensor }; // Using Funky Fresh Encoder

    double m_elbowSensorMeasurements[2] = { 0.0 };
    double m_elbowSensorAverage         = 0.0;

    frc2::PIDController * c_shoulderPid = nullptr;

    Point m_computedGripPoint;

    struct {
        struct {
            struct {
                double lo;
                double hi;
            } limit;

            double zeroOffset;
            double sensorToRadians;
        } turret;

        struct {
            struct {
                double lo;
                double hi;
            } limit;

            double zeroOffset;
        } shoulder;

        struct {
            struct {
                double lo;
                double hi;
            } limit;

            double zeroOffset;
            double sensorToRadians;
        } elbow;

        struct {
            struct {
                double lo;
                double hi;
            } limit;

            double zeroOffset;
            double sensorToRadians;
        } wrist;

        struct {
            struct {
                double lo;
                double hi;
            } limit;

            double zeroOffset;
            double sensorToMeters;

            struct {
                double open;
                double close;
            } setpoint;
        } grip;
    } config;
};
