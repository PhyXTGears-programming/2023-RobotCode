// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "Mandatory.h"

#include <frc/Filesystem.h>

#include <iostream>

#include "subsystems/auto/auto.h"
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "commands/drivetrain/driveTeleopCommand.h"

#include "external/cpptoml.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc/Timer.h>

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
  c_armTeleopCommand = new ArmTeleopCommand(c_arm, c_operatorController);
  c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

  //Auto chooser
  c_chooser.SetDefaultOption(c_autoNameDefault, c_autoNameDefault);
  c_chooser.AddOption(c_autoNameDumpCubeAndScore, c_autoNameDumpCubeAndScore);
  c_chooser.AddOption(c_autoNameDumpScoreAndLeave, c_autoNameDumpScoreAndLeave);
  c_chooser.AddOption(c_autoNameDumpScoreAndLeaveOverChargeStation, c_autoNameDumpScoreAndLeaveOverChargeStation);
  c_chooser.AddOption(c_autoTesting, c_autoTesting);

  frc::SmartDashboard::PutData("Auto Modes", &c_chooser);

  c_autoDumpCubeAndScore = makeAutoDumpCubeAndScore(c_drivetrain);
  c_autoDumpCubeScoreAndLeaveSafeZone = makeAutoDumpCubeAndScoreAndLeaveSafeZone(c_drivetrain);
  c_autoDumpCubeScoreAndLeaveSafeZoneThenLevel = makeAutoDumpCubeAndScoreAndLeaveSafeZoneThenBalance(c_drivetrain);
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
  m_autoSelected = c_chooser.GetSelected();

  // Arm does not appear to get proper shoulder angle at startup.  Hopefully
  // enough time elapsed before auto init for the sensor to send good data.
  c_arm->resetShoulderAngle();

  //done this way to prevent branch misses (because there are none)
  if(m_autoSelected == c_autoNameDumpCubeAndScore){
    c_autoDumpCubeAndScore.Schedule();
  }
  if(m_autoSelected == c_autoNameDumpScoreAndLeave){
    c_autoDumpCubeScoreAndLeaveSafeZone.Schedule();
  }
  if(m_autoSelected == c_autoNameDefault){
    //do nothing
  }
  if(m_autoSelected == c_autoTesting){
    c_autoDumpCubeScoreAndLeaveSafeZoneThenLevel.Schedule();
  }
  // TODO: Make sure to cancel autonomous command in teleop init.
}

void Robot::AutonomousPeriodic() {
  c_drivetrain->Periodic();// update drivetrain no matter what
}

void Robot::TeleopInit() {
  // Arm does not appear to get proper shoulder angle at startup.  Hopefully
  // enough time elapsed before auto init for the sensor to send good data.
  c_arm->resetShoulderAngle();

  // Make sure autonomous command is canceled first.
  // done this way to prevent branch misses (because there are no branches)
  if(m_autoSelected == c_autoNameDumpCubeAndScore){
    c_autoDumpCubeAndScore.Cancel();
  }
  if(m_autoSelected == c_autoNameDumpScoreAndLeave){
    c_autoDumpCubeScoreAndLeaveSafeZone.Cancel();
  }
  if(m_autoSelected == c_autoNameDefault){
    // dont do anything
  }
  if(m_autoSelected == c_autoTesting){
    c_autoDumpCubeScoreAndLeaveSafeZoneThenLevel.Cancel();
  }

  c_armTeleopCommand->Schedule();
  c_armTeleopCommand->resetTarget();
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

  if(c_driverController->GetXButtonPressed()) {
    c_drivetrain->lockMovement(false);
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
