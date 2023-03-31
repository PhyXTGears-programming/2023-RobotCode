// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include "Mandatory.h"

#include <frc/Filesystem.h>

#include <iostream>

#include "commands/arm/MoveToPoint.h"
#include "subsystems/auto/auto.h"
#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "commands/drivetrain/driveTeleopCommand.h"

#include "external/cpptoml.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ScheduleCommand.h>
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
    c_kickstand = new Kickstand();

    //Commands
    c_armTeleopCommand = new ArmTeleopCommand(c_arm, c_operatorController);
    c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);
    c_kickstandReleaseCommand = new KickstandReleaseCommand(c_kickstand);

    //Auto chooser
    c_chooser.SetDefaultOption(c_autoNameDefault, c_autoNameDefault);
    c_chooser.AddOption(c_autoNameDumpCubeAndScore, c_autoNameDumpCubeAndScore);
    c_chooser.AddOption(c_autoNameDumpScoreAndLeave, c_autoNameDumpScoreAndLeave);

    frc::SmartDashboard::PutData("Auto Modes", &c_chooser);

    c_autoDumpCubeAndScore = makeAutoDumpCubeAndScore(c_drivetrain);
    c_autoDumpCubeScoreAndLeaveSafeZone = makeAutoDumpCubeAndScoreAndLeaveSafeZone(c_drivetrain);


    c_armMoveToHome = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getHomePoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToIntake = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getIntakePoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToSubstation = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getSubstationPoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToHybrid = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getHybridPoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToLowPole = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getLowPolePoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToHighPole = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getHighPolePoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToLowShelf = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getLowShelfPoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();

    c_armMoveToHighShelf = frc2::SequentialCommandGroup{
        MoveToPointCommand(c_arm, c_arm->getHighShelfPoint()),
        frc2::ScheduleCommand(c_armTeleopCommand)
    }.ToPtr();
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
        //default auto command
    }
    // TODO: Make sure to cancel autonomous command in teleop init.
    c_drivetrain->enableHeadingControl();
    // c_drivetrain->resetNavxHeading();
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
        c_autoDumpCubeAndScore.Schedule();
    }
    if(m_autoSelected == c_autoNameDumpScoreAndLeave){
        c_autoDumpCubeScoreAndLeaveSafeZone.Schedule();
    }
    if(m_autoSelected == c_autoNameDefault){
        //default auto command
    }

    c_armTeleopCommand->Schedule();
    c_armTeleopCommand->resetTarget();
    c_driveTeleopCommand->Schedule();
    c_drivetrain->enableFieldCentric();
    c_drivetrain->disableHeadingControl();
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

    if(c_driverController->GetYButtonPressed()){
        // c_kickstandReleaseCommand->Schedule();
        c_drivetrain->toggleHeadingControl();
    }

    if (0 == c_operatorController->GetPOV()) {
        // If up d-pad pressed
        c_armTeleopCommand->Cancel();
        c_armMoveToSubstation.Schedule();
    } else if (c_armMoveToSubstation.IsScheduled()) {
        c_armMoveToSubstation.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (90 == c_operatorController->GetPOV()) {
        // If right d-pad pressed
        c_armTeleopCommand->Cancel();
        c_armMoveToHybrid.Schedule();
    } else if (c_armMoveToHybrid.IsScheduled()) {
        c_armMoveToHybrid.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (180 == c_operatorController->GetPOV()) {
        // If right d-pad pressed
        c_armTeleopCommand->Cancel();
        c_armMoveToHome.Schedule();
    } else if (c_armMoveToHome.IsScheduled()) {
        c_armMoveToHome.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (270 == c_operatorController->GetPOV()) {
        // If right d-pad pressed
        c_armTeleopCommand->Cancel();
        c_armMoveToIntake.Schedule();
    } else if (c_armMoveToIntake.IsScheduled()) {
        c_armMoveToIntake.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (c_operatorController->GetAButton()) {
        c_armTeleopCommand->Cancel();
        c_armMoveToLowShelf.Schedule();
    } else if (c_armMoveToLowShelf.IsScheduled()) {
        c_armMoveToLowShelf.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (c_operatorController->GetBButton()) {
        c_armTeleopCommand->Cancel();
        c_armMoveToHighShelf.Schedule();
    } else if (c_armMoveToHighShelf.IsScheduled()) {
        c_armMoveToHighShelf.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (c_operatorController->GetXButton()) {
        c_armTeleopCommand->Cancel();
        c_armMoveToLowPole.Schedule();
    } else if (c_armMoveToLowPole.IsScheduled()) {
        c_armMoveToLowPole.Cancel();
        c_armTeleopCommand->Schedule();
    }

    if (c_operatorController->GetYButton()) {
        c_armTeleopCommand->Cancel();
        c_armMoveToHighPole.Schedule();
    } else if (c_armMoveToHighPole.IsScheduled()) {
        c_armMoveToHighPole.Cancel();
        c_armTeleopCommand->Schedule();
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
