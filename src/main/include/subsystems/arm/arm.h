#pragma once

#include "Mandatory.h"
#include "subsystems/arm/armPose.h"
#include "subsystems/arm/boundary.h"
#include "subsystems/arm/motionPath.h"
#include "util/geom.h"

#include "external/cpptoml.h"

#include <cmath>
#include <memory>

#include <frc/AnalogPotentiometer.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/CANSparkMax.h>

using namespace Interfaces::Arm;

class ArmSubsystem : public frc2::SubsystemBase {
public:
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

    Point const & getSafetyPoint(Point pt);

    Point const & getIntakePoint();
    Point const & getHomePoint();
    Point const & getHybridPoint();
    Point const & getLowPolePoint();
    Point const & getHighPolePoint();
    Point const & getLowShelfPoint();
    Point const & getHighShelfPoint();

    void setTurretAngle(double angle);
    void setShoulderAngle(double angle);
    void setElbowAngle(double angle);
    void setWristRollAngle(double angle);
    void setGrip(double grip); // hehe grippy grabby hand

    /**
     * Given a target position, compute necessary joint angles, and move joints
     * of arm toward target.  Will NOT stop on its own.  Must call this method
     * repeatedly to stop on target.
     *
     * @return false if point is within the no-go zone, true if point is safe.
     */
    bool moveToPoint(Point const & target);

private:
    void loadConfig(std::shared_ptr<cpptoml::table> toml);

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

    rev::CANSparkMax m_turretMotor {
        k_TurretMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax m_lowJointMotor {
        k_LowJointMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax m_midJointMotor {
        k_MidJointMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax m_gripperRotateMotor {
        k_GripperRotateMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };
    rev::CANSparkMax m_gripperGraspMotor {
        k_GripperGraspMotor,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };

    frc::AnalogPotentiometer m_turretAngleSensor {k_TurretSensor, 1.0, 0.0};
    frc::AnalogPotentiometer m_elbowAngleSensor {k_ElbowSensor, 1.0, 0.0};
    frc::AnalogPotentiometer m_wristRollAngleSensor {k_WristRollSensor, 1.0, 0.0};
    frc::AnalogPotentiometer m_gripSensor {k_GripSensor, 1.0, 0.0};
    frc::DutyCycleEncoder m_shoulderAngleSensor{k_ShoulderSensor}; // Using Funky Fresh Encoder

    frc2::PIDController * m_shoulderPid = nullptr;

    std::shared_ptr<Boundary> m_noGoZone = nullptr;

    // Safety Points
    Point m_safetyPointGrid{-0.2540, 0.0, 0.9144};
    Point m_safetyPointCenter{0.0, 0.254, 0.8635};
    Point m_safetyPointIntake{0.254, 0.0, -0.8128};

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

public:
    // Other Points
    Point m_pointIntake{0.4064, 0, 0.1524};
    Point m_pointHome{0.0, 0.0, 0.0}; //   ! !  Unknown  ! !
    Point m_pointHybrid{-0.5715, 0.0, 0.2286};
    Point m_pointLowPole{-0.9589, 0.0, 1.0668};
    Point m_pointHighPole{-1.3907, 0.0, 1.2700};
    Point m_pointLowShelf{-0.9398, 0.0, 0.7620};
    Point m_pointHighShelf{-1.3335, 0.0, 1.0668};
};
