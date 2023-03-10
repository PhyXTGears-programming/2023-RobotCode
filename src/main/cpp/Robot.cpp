// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Mandatory.h"

#include <frc/Filesystem.h>

#include <iostream>
#include <sstream>

#include "subsystems/drivetrain/drivetrain.h"
#include "subsystems/drivetrain/odometry.h"
#include "commands/drivetrain/driveTeleopCommand.h"

#include "util/math.h"

#include "external/cpptoml.h"

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
  c_drivetrain = new Drivetrain(false);
  c_odometry = new Odometry(c_drivetrain);
  c_arm = new ArmSubsystem(c_toml->get_table("arm"));

  //Commands
  c_driveTeleopCommand = new DriveTeleopCommand(c_drivetrain, c_driverController);

  m_gripTarget = c_arm->getGripPoint();
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

    std::stringstream gripTargetStr;

    gripTargetStr << std::fixed << std::setprecision(4)
        << "(" << m_gripTarget.x
        << ", " << m_gripTarget.y
        << ", " << m_gripTarget.z
        << ")";
    frc::SmartDashboard::PutString("Test Pos (m)", gripTargetStr.str());
    frc::SmartDashboard::PutBoolean("Test Point Is Safe", c_arm->isPointSafe(m_gripTarget));

    // Report calculated arm pose angles.  What the robot thinks the angles
    // should be to reach gripper position.

    ArmPose pose = c_arm->calcIKJointPoses(m_gripTarget);
    frc::SmartDashboard::PutNumber("Test Turret Angle (deg)",   RAD_2_DEG(pose.turretAngle));
    frc::SmartDashboard::PutNumber("Test Shoulder Angle (deg)", RAD_2_DEG(pose.shoulderAngle));
    frc::SmartDashboard::PutNumber("Test Elbow Angle (deg)",    RAD_2_DEG(pose.elbowAngle));
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
  // TODO: Make sure to cancel autonomous command in teleop init.
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // TODO: Make sure autonomous command is canceled first.
  //c_driveTeleopCommand->Schedule();

  m_gripTarget = c_arm->getGripPoint();

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

    const double maxPointSpeed = 0.005;
    const double maxPointRotSpeed = 2.0 * M_PI / 10.0 * 0.02;   // radians per second in 20ms.
    const double maxWristRotSpeed = 2.0 * M_PI / 2.0 * 0.2;
    const double maxGripSpeed = 0.1;

    Vector gripVec(m_gripTarget.x, m_gripTarget.y, m_gripTarget.z);
    double gripMag = gripVec.len();
    double gripDir = std::atan2(gripVec.x, gripVec.y);  // 0 deg == y axis

    // Rotate turret. Speed of rotation is reduced the further the arm reaches.
    double leftX = c_operatorController->GetLeftX();
    leftX = std::copysign(std::clamp(leftX * leftX, -1.0, 1.0), leftX);
    leftX = std::abs(leftX) < 0.1 ? 0.0 : leftX;
    if (0.0 != leftX) {
        // (+) leftX should move turret clockwise.
        gripDir = gripDir + leftX * maxPointRotSpeed / gripMag;
        Vector offset(
            gripMag * std::cos(gripDir),
            gripMag * std::sin(gripDir),
            0.0
        );
        m_gripTarget = m_gripTarget + offset;
    }

    // Extend/retract gripper from/to turret.
    double leftY = -c_operatorController->GetLeftY(); /* Invert so + is forward */
    leftY = std::copysign(std::clamp(leftY * leftY, -1.0, 1.0), leftY);
    leftY = std::abs(leftY) < 0.1 ? 0.0 : leftY;
    if (0.0 != leftY) {
        // (+) leftY should move away from turret.
        gripMag = gripMag + leftY * maxPointSpeed;
        Vector offset(
            gripMag * std::cos(gripDir),
            gripMag * std::sin(gripDir),
            0.0
        );
        m_gripTarget = m_gripTarget + offset;
    }

    // Move gripper up or down.
    double rightY = -c_operatorController->GetRightY();    /* Invert so + is up */
    rightY = std::copysign(std::clamp(rightY * rightY, -1.0, 1.0), rightY);
    rightY = std::abs(rightY) < 0.1 ? 0.0 : rightY;
    if (0.0 != rightY) {
        // (+) rightY should move gripper up.
        m_gripTarget = m_gripTarget + Vector(0.0, 0.0, rightY * maxPointSpeed);
    }

    // FIXME: c_arm->moveToPoint(m_gripTarget);

    // Rotate wrist clockwise or counterclockwise.
    {
        double leftTrigger = c_operatorController->GetLeftTriggerAxis();
        double rightTrigger = c_operatorController->GetRightTriggerAxis();
        double trigger = leftTrigger - rightTrigger;
        trigger = std::copysign(std::clamp(trigger * trigger, -1.0, 1.0), trigger);
        trigger = std::abs(trigger) < 0.1 ? 0.0 : trigger;

        // Use current angle as default so wrist doesn't move wildly upon enable.
        double wristTargetAngle = c_arm->getWristRollAngle();

        if (0.0 != trigger) {
            // (+) trigger should rotate wrist counter-clockwise, looking down forearm.
            wristTargetAngle += trigger * maxWristRotSpeed;
        }

        // Move/hold wrist angle.
        c_arm->setWristRollAngle(wristTargetAngle);
    }

    // Open and close gripper.
    {
        double bumper =
            (c_operatorController->GetRightBumper() ? 1.0 : 0.0)
            - (c_operatorController->GetLeftBumper() ? 1.0 : 0.0);

        // Use current position as default so grip doesn't move wildly upon enable.
        double gripTargetPos = c_arm->getGrip();

        if (0.0 != bumper) {
            gripTargetPos += bumper * maxGripSpeed;
        }

        // Move/hold grip.
        c_arm->setGrip(gripTargetPos);
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
