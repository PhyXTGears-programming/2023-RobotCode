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

#ifdef COMPETITION_MODE
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc/Timer.h>
#endif

void Robot::RobotInit() {
  try{
    c_toml = cpptoml::parse_file(frc::filesystem::GetDeployDirectory()+"/config.toml");
  } catch (cpptoml::parse_exception & ex){
    std::cerr << "Unable to open file: config.toml" << std::endl;
    exit(1);
  }
  
  //HIDs
  c_driverController = new frc::XboxController(Interfaces::k_driverXboxController);
  c_operatorController = new frc::XboxController(Interfaces::k_operatorXboxController);

  //Subsystems
  c_drivetrain = new Drivetrain(false);
  c_odometry = new Odometry(c_drivetrain);

  //Commands
  c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

  //temp auto
  #ifdef COMPETITION_MODE
  timerOne = new frc::Timer();
  timerTwo = new frc::Timer();
  outOfSafeZone = new frc2::FunctionalCommand{
    [&](){timerOne->Reset();},
    [&](){c_drivetrain->setMotion(0,0.5,0);},
    [&](bool interrupted){c_drivetrain->setMotion(0,0,0);},
    [&](){return timerOne->AdvanceIfElapsed(3.0_s);},
    {c_drivetrain}
  };
  ontoPlatform = new frc2::FunctionalCommand{
    [&](){timerTwo->Reset();},
    [&](){c_drivetrain->setMotion(0,-0.5,0);},
    [&](bool interrupted){c_drivetrain->setMotion(0,0,0);},
    [&](){return timerTwo->AdvanceIfElapsed(2.0_s);},
    {c_drivetrain}
  };
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
  outOfSafeZone->Schedule();
  // TODO: Make sure to cancel autonomous command in teleop init.
}

void Robot::AutonomousPeriodic() {
  if(!outOfSafeZone->IsScheduled()){ // check to see if driving out of the save zone is still running (don't want to drive in opposite directions at the same time)
    if(!ontoPlatform->IsScheduled()){ // check to see if we have started the movement onto the platform (should not start it multiple times)
      ontoPlatform->Schedule();
    }
  }
  c_drivetrain->Periodic();// update drivetrain no matter what
}

void Robot::TeleopInit() {
  // TODO: Make sure autonomous command is canceled first.
  c_driveTeleopCommand->Schedule();
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
