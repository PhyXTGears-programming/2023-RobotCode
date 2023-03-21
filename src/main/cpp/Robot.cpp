// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/XboxController.h>

#include "Mandatory.h"
#include "subsystems/arm/arm.h"
#include "util/math.h"

#include <frc/Filesystem.h>

#include <iostream>

#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "commands/drivetrain/driveTeleopCommand.h"

#include "external/cpptoml.h"

#include <frc/smartdashboard/SmartDashboard.h>

#ifdef COMPETITION_MODE
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc/Timer.h>
#endif

void Robot::RobotInit() {
  try{
    c_toml = cpptoml::parse_file(frc::filesystem::GetDeployDirectory()+"/config.toml");
  } catch (cpptoml::parse_exception & ex){
    std::cerr << "Unable to open file: config.toml"
        << std::endl
        << ex.what()
        << std::endl;
    exit(1);
  }
  
  //HIDs
  c_driverController = new frc::XboxController(Interfaces::k_driverXboxController);
  c_operatorController = new frc::XboxController(Interfaces::k_operatorXboxController);

  //Subsystems
  c_drivetrain = new Drivetrain(true);
  c_odometry = new Odometry(c_drivetrain);
  c_arm = new ArmSubsystem(c_toml->get_table("arm"));

  //Commands
  //c_armTeleopCommand = new ArmTeleopCommand(c_arm, c_operatorController);
  c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

  //temp auto
  #ifdef COMPETITION_MODE
  auto orientWheels = frc2::StartEndCommand{
    [&] () { c_drivetrain->setMotion(0,0.05,0); },
    [&] () { c_drivetrain->setMotion(0,0,0); },
    { c_drivetrain }
  }.ToPtr();
  auto forceOffCube = frc2::StartEndCommand{
    [&] () { c_drivetrain->setMotion(0,0.5,0); },
    [&] () { c_drivetrain->setMotion(0,0,0); },
    { c_drivetrain }
  }.ToPtr();

  auto putCubeIntoStation = frc2::StartEndCommand{
    [&] () { c_drivetrain->setMotion(0,-0.15,0); },
    [&] () { c_drivetrain->setMotion(0,0,0); },
    {c_drivetrain}
  }.ToPtr();

  c_simpleAuto = std::move(orientWheels).WithTimeout(0.5_s)
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

  frc::SmartDashboard::PutBoolean("Field Centric Enabled",c_drivetrain->getFieldCentric());
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
  //c_drivetrain->Periodic();// update drivetrain no matter what
}

void Robot::TeleopInit() {
  // Make sure autonomous command is canceled first.
  c_simpleAuto->Cancel();

  //c_armTeleopCommand->Schedule();
  //c_armTeleopCommand->resetTarget();
  //c_driveTeleopCommand->Schedule();
  //c_drivetrain->enableFieldCentric();
}
/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  #define DEAD_ZONE 0.10
  #define INPUT(x, min, max) ((std::abs((x)) < DEAD_ZONE) \
    ? 0.0 \
    : ((x) + std::copysign(1.0 - DEAD_ZONE, (x))) \
      / (1.0 - DEAD_ZONE) \
      * ((max) - (min)) \
      + std::copysign((min), (x)))

  double leftX = c_driverController->GetLeftX();
  double leftY = c_driverController->GetLeftY();
  double rightY = -c_driverController->GetRightY();

  // 1. Run first
  frc::SmartDashboard::PutNumber("Left X", leftX);
  frc::SmartDashboard::PutNumber("Left Y", leftY);
  frc::SmartDashboard::PutNumber("Right Y", rightY);
  frc::SmartDashboard::PutNumber("Left Trigger", c_driverController->GetLeftTriggerAxis());


  // 2. Test Turret
  //    - Invert direction (CW +):    ??
  //    - Zero offset:    ??
  //    - -90 (left):     ??
  //    -  90 (right):    ??
  //    - left max:       ??
  //    - right max:      ??
  leftX = INPUT(leftX, 0.01, 0.05);
  c_arm->setTurretSpeed(leftX);

  // 3. Test Shoulder
  //    - Invert direction (up is z+):  ??
  //    - Zero offset (z+):   ??
  //    - -45 deg (down):     ??
  //    - down max:           ??
  //    - up max:             ??
  double boost = (leftY < 0.0) ? 0.050 : 0.0;
  leftY = INPUT(leftY, 0.035 + boost, 0.035 + boost);
  c_arm->setShoulderSpeed(leftY);


  // 4. Test Elbow
  //    - Invert direction (up is z+):  ??
  //    - Zero offset (z+):   ??
  //    - 90 deg (up):     ??
  //    - down max:           ??
  //    - up max:             ??
  rightY = INPUT(rightY, 0.05, 0.15);
  c_arm->setElbowSpeed(rightY);

  // 5. Test Wrist Roll
  //    - Invert direction (cw is +):  ??
  //    - Zero offset (min ccw):       ??
  //    - 180 deg (cw):       ??
  //    - down max:           ??
  //    - up max:             ??
  leftX = c_operatorController->GetLeftX();
  leftX = INPUT(leftX, 0.01, 0.05);
  c_arm->setWristRollSpeed(leftX);

  // 6. Test Grip
  //    - Invert direction (open is +):  ??
  //    - Zero offset (closed):   ??
  //    - open distance (m):      ??
  //    - down max:               ??
  //    - up max:                 ??
  double rightX = c_operatorController->GetRightX();
  rightX = INPUT(rightX, 0.05, 0.1);
  c_arm->setGripSpeed(rightX);
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
