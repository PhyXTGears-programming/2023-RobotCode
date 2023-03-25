#include "Mandatory.h"

#include "commands/drivetrain/driveLevelOnChargeStation.h"

#include "subsystems/drivetrain/drivetrain.h"

#include "commands/drivetrain/autoBalance.h"

DriveLevelCommand::DriveLevelCommand(Drivetrain* drivetrain) {
    c_drivetrain = drivetrain;

    c_autoBalance = new autoBalance(c_drivetrain);
    AddRequirements(c_drivetrain);
}


void DriveLevelCommand::Initialize() {
    c_drivetrain->setMotion(0, 0, 0); // make sure nothing is moving
}

void DriveLevelCommand::Execute() {
    double tiltMagnitude = std::sqrt( std::pow(c_drivetrain->getXTilt(), 2) + std::pow(c_drivetrain->getYTilt(), 2) ); // gets the angle from the floor
    double movementSpeed = 0;

    c_drivetrain->setMotion(0, movementSpeed, 0)

}

// void DriveLevelCommand::Execute() {
//     c_drivetrain->setMotion(0, c_autoBalance->autoBalanceRoutine(), 0); // just move forwards
// }

void DriveLevelCommand::End(bool interrupted){
    // set the movement to 0 just as a safety
    c_drivetrain->setMotion(0, 0, 0);
}
/*
bool DriveLevelCommand::IsFinished(){
    return (std::min(Constants::k_levelTolerance, std::abs(c_drivetrain->getXTilt())) < Constants::k_levelTolerance) && 
           (std::min(Constants::k_levelTolerance, std::abs(c_drivetrain->getYTilt())) < Constants::k_levelTolerance);
}*/

bool DriveLevelCommand::IsFinished(){
    return c_autoBalance->autoBalanceRoutine() == 0;
}

