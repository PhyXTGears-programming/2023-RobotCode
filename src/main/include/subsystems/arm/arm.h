#pragma once

#include "Mandatory.h"
#include "subsystems/arm/armPose.h"
#include "subsystems/arm/boundary.h"
#include "subsystems/arm/motionPath.h"
#include "util/geom.h"

#include "external/cpptoml.h"

#include <cmath>
#include <memory>
#include <optional>

#include <frc/AnalogPotentiometer.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/Servo.h>
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
    Point calcElbowPos(double turretAng, double angShoulder);

    Point calcGripPos(double turretAng, double shoulderAng, double elbowAng);

    ArmPose calcIKJointPoses(Point const & pt);

    bool isPointSafe(Point const & point);
    bool isNearPoint(Point const & point);

    MotionPath getPathTo(Point const & current, Point const & target);

    // Reading Functions:
    double getTurretAngle();
    double getShoulderAngle();
    double getElbowAngle();
    double getWristRollAngle();
    double getGrip();

    // Getting Points:
    Point getGripPoint();

    SafetyZone getSafetyZone(Point const & pt);

    Point const & getIntakePoint();
    Point const & getHomePoint();
    Point const & getSubstationPoint();
    Point const & getHybridPoint();
    Point const & getLowPolePoint();
    Point const & getHighPolePoint();
    Point const & getLowShelfPoint();
    Point const & getHighShelfPoint();

    Point const & getCenterSafePoint();
    Point const & getIntakeSafePoint();
    Point const & getGridSafePoint();

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
    std::optional<Point> moveToPoint(Point const & target);

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

    rev::CANSparkMax c_turretMotor {
        k_TurretMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax c_lowJointMotor {
        k_LowJointMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax c_midJointMotor {
        k_MidJointMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax c_gripperRotateMotor {
        k_GripperRotateMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax c_gripperGraspMotor {
        k_GripperGraspMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };

    frc::Servo c_cameraServo { k_CameraServo };

    frc::AnalogPotentiometer c_turretAngleSensor {k_TurretSensor, 1.0, 0.0};
    frc::AnalogPotentiometer c_elbowAngleSensor {k_ElbowSensor, 1.0, 0.0};
    frc::AnalogPotentiometer c_wristRollAngleSensor {k_WristRollSensor, 1.0, 0.0};
    frc::AnalogPotentiometer c_gripSensor {k_GripSensor, 1.0, 0.0};
    frc::DutyCycleEncoder c_shoulderAngleSensor{k_ShoulderSensor}; // Using Funky Fresh Encoder

    double m_elbowSensorMeasurements[2] = { 0.0 };
    double m_elbowSensorAverage = 0.0;

    frc2::PIDController * c_shoulderPid = nullptr;

    std::shared_ptr<Boundary> c_noGoZone = nullptr;

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

    // Safety Points
    Point c_safetyPointGrid{0.53, 0.03, 0.70};
    Point c_safetyPointCenter{0.0, 0.657, 0.68};
    Point c_safetyPointIntake{-0.53, 0.03, 0.68};

    // Other Points
    Point c_pointIntake{0.4064, 0, 0.1524};
    Point c_pointHome{0.020, 0.140, 0.386};
    Point c_pointSubstation{0.0, 0.0, 1.2700};
    Point c_pointHybrid{-0.5715, 0.0, 0.2286};
    Point c_pointLowPole{-0.9589, 0.0, 1.0668};
    Point c_pointHighPole{-1.3907, 0.0, 1.2700};
    Point c_pointLowShelf{-0.9398, 0.0, 0.7620};
    Point c_pointHighShelf{-1.3335, 0.0, 1.0668};
};
