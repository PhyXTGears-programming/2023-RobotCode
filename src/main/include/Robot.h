// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "Mandatory.h"

#include <frc/XboxController.h>

#include "subsystems/arm/arm.h"
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "commands/drivetrain/driveTeleopCommand.h"

#include "external/cpptoml.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  std::shared_ptr<cpptoml::table> c_toml;

  //HIDs
  frc::XboxController* c_driverController;
  frc::XboxController* c_operatorController;

  //Subsystems
  Drivetrain* c_drivetrain = nullptr;
  Odometry* c_odometry = nullptr;
  ArmSubsystem* c_arm = nullptr;

  //Commands
  DriveTeleopCommand* c_driveTeleopCommand = nullptr;

  Point m_gripTarget;
};
