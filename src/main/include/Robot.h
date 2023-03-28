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

#include "commands/arm/ArmTeleopCommand.h"
#include "commands/drivetrain/driveTeleopCommand.h"
#include "commands/kickstand/kickstandReleaseCommand.h"

#include "external/cpptoml.h"

//temporary auto
#include <frc2/command/CommandBase.h>
#include <frc2/command/StartEndCommand.h>

//auto chooser
#include <frc/smartdashboard/SendableChooser.h>

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
        Kickstand* c_kickstand = nullptr;

        //Commands
        ArmTeleopCommand* c_armTeleopCommand = nullptr;
        DriveTeleopCommand* c_driveTeleopCommand = nullptr;
        KickstandReleaseCommand* c_kickstandReleaseCommand = nullptr;

        //Auto Selector
        std::string m_autoSelected;
        frc::SendableChooser<std::string> c_chooser;
        const std::string c_autoNameDefault = "Default";
        const std::string c_autoNameDumpCubeAndScore = "Dump Cube and Score";
        const std::string c_autoNameDumpScoreAndLeave = "Dump Cube, Score, Then Leave Safe Zone";

        //dump cube and put into scoring zone auto
        frc2::CommandPtr c_autoDumpCubeAndScore{nullptr};

        //dump cube and put into scoring zone auto, then leave safe zone
        frc2::CommandPtr c_autoDumpCubeScoreAndLeaveSafeZone{nullptr};
};
