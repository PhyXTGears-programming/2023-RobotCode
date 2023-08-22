// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include "Mandatory.h"

#include <frc/Filesystem.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandScheduler.h>

#include <wpinet/uv/Timer.h>

#include <iostream>

#include "commands/drivetrain/driveTeleopCommand.h"
#include "subsystems/arm/arm.h"
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "util/calibrate/RemoteCalibrateService.h"

#include "external/cpptoml.h"

#ifdef COMPETITION_MODE
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#endif

void Robot::RobotInit() {
    try {
        c_toml = cpptoml::parse_file(frc::filesystem::GetDeployDirectory() + "/config.toml");
    } catch (cpptoml::parse_exception & ex) {
        std::cerr << "Unable to open file: config.toml" << std::endl << ex.what() << std::endl;
        exit(1);
    }

    // HIDs
    c_driverController   = new frc::XboxController(Interfaces::k_driverXboxController);
    c_operatorController = new frc::XboxController(Interfaces::k_operatorXboxController);

    // Subsystems
    c_drivetrain = new Drivetrain(true);
    c_odometry   = new Odometry(c_drivetrain);
    // c_arm = new ArmSubsystem(c_toml->get_table("arm"));

    // Commands
    // c_armTeleopCommand = new ArmTeleopCommand(c_arm, c_operatorController);
    c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

// temp auto
#ifdef COMPETITION_MODE
    auto orientWheels = frc2::StartEndCommand{
        [&]() { c_drivetrain->setMotion(0, 0.05, 0); },
        [&]() { c_drivetrain->setMotion(0, 0, 0); },
        {c_drivetrain}}.ToPtr();
    auto forceOffCube = frc2::StartEndCommand{
        [&]() { c_drivetrain->setMotion(0, 0.5, 0); },
        [&]() { c_drivetrain->setMotion(0, 0, 0); },
        {c_drivetrain}}.ToPtr();

    auto putCubeIntoStation = frc2::StartEndCommand{
        [&]() { c_drivetrain->setMotion(0, -0.15, 0); },
        [&]() { c_drivetrain->setMotion(0, 0, 0); },
        {c_drivetrain}}.ToPtr();

    c_simpleAuto = std::move(orientWheels)
                       .WithTimeout(0.5_s)
                       .AndThen(std::move(forceOffCube).WithTimeout(0.3_s))
                       .AndThen(std::move(putCubeIntoStation).WithTimeout(2.0_s))
                       .Unwrap();
#endif

    /// Configure remote calibration service.
    std::shared_ptr<RemoteCalibrationService> mCalibrationService =
        RemoteCalibrationService::Create(wpi::uv::Loop::GetDefault());

    static CalibrationState calibrationState;

    mCalibrationService->onSetVelocityRequest = [&calibrationState](SetVelocityPayload && payload) {
        // Update velocities, and reset safety counters.
        // Timeouts will automatically stop motors/actuators if a set velocity payload fails to
        // arrive within the expected window of time.  If comms are lost, we don't want the robot to
        // run amok.
        calibrationState.lowJoint.velocity        = payload.lowJointVelocity;
        calibrationState.lowJoint.ticksUntilReset = (payload.lowJointVelocity != 0.0) ? 3 : 0;

        calibrationState.midJoint.velocity        = payload.midJointVelocity;
        calibrationState.midJoint.ticksUntilReset = (payload.midJointVelocity != 0.0) ? 3 : 0;

        calibrationState.wristRoll.velocity        = payload.wristRollVelocity;
        calibrationState.wristRoll.ticksUntilReset = (payload.wristRollVelocity != 0.0) ? 3 : 0;

        calibrationState.gripper.velocity        = payload.gripperVelocity;
        calibrationState.gripper.ticksUntilReset = (payload.gripperVelocity != 0.0) ? 3 : 0;
    };

    std::shared_ptr<wpi::uv::Timer> mTickResetTimer =
        wpi::uv::Timer::Create(wpi::uv::Loop::GetDefault());

    mTickResetTimer->timeout.connect([&calibrationState]() {
        // Decrement safety counters.  When safety counters reach zero, stop the motor.  This is a
        // safety mechanism to automatically stop motors in the event the remote control software
        // requests movement, but then loses comms and cannot tell the robot to stop the motor.
        if (0 == calibrationState.lowJoint.ticksUntilReset) {
            calibrationState.lowJoint.velocity = 0.0;
        } else {
            calibrationState.lowJoint.ticksUntilReset -= 1;
        }

        if (0 == calibrationState.midJoint.ticksUntilReset) {
            calibrationState.midJoint.velocity = 0.0;
        } else {
            calibrationState.midJoint.ticksUntilReset -= 1;
        }

        if (0 == calibrationState.wristRoll.ticksUntilReset) {
            calibrationState.wristRoll.velocity = 0.0;
        } else {
            calibrationState.wristRoll.ticksUntilReset -= 1;
        }

        if (0 == calibrationState.gripper.ticksUntilReset) {
            calibrationState.gripper.velocity = 0.0;
        } else {
            calibrationState.gripper.ticksUntilReset -= 1;
        }
    });

    mTickResetTimer->Start(
        std::chrono::duration<uint64_t, std::milli>(20),
        std::chrono::duration<uint64_t, std::milli>(20)
    );
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();

    frc::SmartDashboard::PutBoolean("Field Centric Enabled", c_drivetrain->getFieldCentric());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    c_simpleAuto->Schedule();
    // TODO: Make sure to cancel autonomous command in teleop init.
}

void Robot::AutonomousPeriodic() {
    c_drivetrain->Periodic(); // update drivetrain no matter what
}

void Robot::TeleopInit() {
    // TODO: Make sure autonomous command is canceled first.
    // c_driveTeleopCommand->Schedule();
}

void Robot::TeleopInit() {
    // Make sure autonomous command is canceled first.
    c_simpleAuto->Cancel();

    // c_armTeleopCommand->Schedule();
    // c_armTeleopCommand->resetTarget();
    c_driveTeleopCommand->Schedule();
    c_drivetrain->enableFieldCentric();
}
/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    static const double DEAD_ZONE = 0.05;

    // (+) is right on x axis.
    double leftX = c_driverController->GetLeftX();
    // Invert y axis so (+) is up.
    double leftY = -c_driverController->GetLeftY();

    // 1. Run first
    frc::SmartDashboard::PutNumber("Left X", leftX);
    frc::SmartDashboard::PutNumber("Left Y", leftY);
    frc::SmartDashboard::PutNumber("Left Trigger", c_driverController->GetLeftTriggerAxis());

    // 2. Test Turret
    //    - Invert direction (CW +):    ??
    //    - Zero offset:    ??
    //    - -90 (left):     ??
    //    -  90 (right):    ??
    //    - left max:       ??
    //    - right max:      ??
    // if (abs(leftX) < DEAD_ZONE) {
    //   arm->setTurretSpeed(0.0);
    // } else {
    //   arm->setTurretSpeed(0.1 * leftX);
    // }

    // 3. Test Shoulder
    //    - Invert direction (up is z+):  ??
    //    - Zero offset (z+):   ??
    //    - -45 deg (down):     ??
    //    - down max:           ??
    //    - up max:             ??
    // if (abs(leftY) < DEAD_ZONE) {
    //   arm->setShoulderSpeed(0.0);
    // } else {
    //   arm->setShoulderSpeed(0.1 * leftY);
    // }

    // 4. Test Elbow
    //    - Invert direction (up is z+):  ??
    //    - Zero offset (z+):   ??
    //    - 90 deg (up):     ??
    //    - down max:           ??
    //    - up max:             ??
    // if (abs(leftY) < DEAD_ZONE) {
    //   arm->setElbowSpeed(0.0);
    // } else {
    //   arm->setElbowSpeed(0.1 * leftY);
    // }

    // 5. Test Wrist Roll
    //    - Invert direction (cw is +):  ??
    //    - Zero offset (min ccw):       ??
    //    - 180 deg (cw):       ??
    //    - down max:           ??
    //    - up max:             ??
    // if (abs(leftX) < DEAD_ZONE) {
    //   arm->setWristRollSpeed(0.0);
    // } else {
    //   arm->setWristRollSpeed(0.1 * leftX);
    // }

    // 6. Test Grip
    //    - Invert direction (open is +):  ??
    //    - Zero offset (closed):   ??
    //    - open distance (m):      ??
    //    - down max:               ??
    //    - up max:                 ??
    // if (abs(leftX) < DEAD_ZONE) {
    //   arm->setGripperGraspSpeed(0.0);
    // } else {
    //   arm->setGripperGraspSpeed(0.1 * leftX);
    // }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
