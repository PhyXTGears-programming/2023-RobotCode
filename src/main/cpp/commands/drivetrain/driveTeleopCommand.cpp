#include "Mandatory.h"

#include "commands/drivetrain/driveTeleopCommand.h"

#include "subsystems/drivetrain/drivetrain.h"
#include <frc/XboxController.h>

#define JOYSTICK_DEADZONE 0.1
#define DEADZONE(input) ((fabs(input) < JOYSTICK_DEADZONE) ? 0.0 : input)

DriveTeleopCommand::DriveTeleopCommand(Drivetrain* drivetrain, frc::XboxController* driverController){
    c_driverController = driverController;
    c_drivetrain = drivetrain;

    AddRequirements(c_drivetrain);
}

void DriveTeleopCommand::Initialize(){
    c_drivetrain->setMotion(0,0,0); //make sure nothing is moving
}

void DriveTeleopCommand::Execute(){
    // compresses the range of the driving speed to be within the max speed and the minimum, but have the normal speed
    // be the default if no trigger is being pressed (so both register 0)
    // 
    // NOTE: no trigger takes priority of the other, so if both pressed, they will cancel each other
    double reduce = c_driverController->GetLeftTriggerAxis() * (Constants::k_normalDriveSpeed - Constants::k_minDriveSpeed);
    double gain = c_driverController->GetRightTriggerAxis() * (Constants::k_maxDriveSpeed - Constants::k_normalDriveSpeed);

    double speedFactor = Constants::k_normalDriveSpeed + gain - reduce;

    // the rotation limit is there in case the driver does not want to spin as fast while driving (specifically limiting the controller input)
    c_drivetrain->setMotion(-DEADZONE(c_driverController->GetLeftX())*speedFactor, DEADZONE(c_driverController->GetLeftY())*speedFactor, DEADZONE(c_driverController->GetRightX())*Constants::k_maxSpinSpeed);
}

void DriveTeleopCommand::End(bool interrupted){
    // set the movement to 0 just as a safety
    c_drivetrain->setMotion(0,0,0);
}

bool DriveTeleopCommand::IsFinished(){
    return false; // dont end because then we wont be able to drive
}