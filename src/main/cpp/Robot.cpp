// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include "Mandatory.h"
#include "subsystems/arm/arm.h"

#include <frc/Filesystem.h>

#include <iostream>

#include "commands/drivetrain/driveTeleopCommand.h"
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"

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

    double leftX = c_driverController->GetLeftX();
    double leftY = c_driverController->GetLeftY();

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
