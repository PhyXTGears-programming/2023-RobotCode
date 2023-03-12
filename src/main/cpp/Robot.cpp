// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "Mandatory.h"

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
  //c_arm = new ArmSubsystem(c_toml->get_table("arm"));

  //Commands
  //c_armTeleopCommand = new ArmTeleopCommand(c_arm, c_operatorController);
  c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

  //temp auto
  #ifdef COMPETITION_MODE
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

  c_simpleAuto = std::move(forceOffCube).WithTimeout(0.5_s)
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
  c_drivetrain->Periodic();// update drivetrain no matter what
}

void Robot::TeleopInit() {
  // Make sure autonomous command is canceled first.
  c_simpleAuto->Cancel();

  //c_armTeleopCommand->Schedule();
  //c_armTeleopCommand->resetTarget();
  c_driveTeleopCommand->Schedule();
  c_drivetrain->enableFieldCentric();
}
/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // Driver A button -> toggle field centric.
  if (c_driverController->GetAButtonPressed()) {
    c_drivetrain->toggleFieldCentric();
  }

  // Driver B button -> reset navx heading.
  if (c_driverController->GetBButtonPressed()) {
    c_drivetrain->resetNavxHeading();
  }
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
