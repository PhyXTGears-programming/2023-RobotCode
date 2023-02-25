#include "Mandatory.h"

#include "commands/drivetrain/driveTeleopCommand.h"

#include "subsystems/drivetrain/drivetrain.h"
#include <frc/XboxController.h>

#define JOYSTICK_DEADZONE 0.1
#define DEADZONE(input) ((fabs(input) < JOYSTICK_DEADZONE) ? 0.0 : input)

DriveTeleopCommand::DriveTeleopCommand(Drivetrain* drivetrain, frc::XboxController* driverController){
    c_driverController = driverController;
    c_drivetrain = drivetrain;
}

void DriveTeleopCommand::Initialize(){
    c_drivetrain->setMotion(0,0,0); //make sure nothing is moving
}

void DriveTeleopCommand::Execute(){
    //default 30% speed, but can go to 60% with full Right trigger press or 15% for full Left trigger press
    double limiter = (1+c_driverController->GetRightTriggerAxis())*(1/(1+c_driverController->GetLeftTriggerAxis()))*0.3;
    c_drivetrain->setMotion(-DEADZONE(c_driverController->GetLeftX())*limiter, DEADZONE(c_driverController->GetLeftY())*limiter, DEADZONE(c_driverController->GetRightX()));
}

void DriveTeleopCommand::End(bool interrupted){
    // set the movement to 0 just as a safety
    c_drivetrain->setMotion(0,0,0);
}

bool DriveTeleopCommand::IsFinished(){
    return false; // dont end because then we wont be able to drive
}