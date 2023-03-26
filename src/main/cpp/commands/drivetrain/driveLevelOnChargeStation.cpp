#include "Mandatory.h"

#include "commands/drivetrain/driveLevelOnChargeStation.h"

#include "subsystems/drivetrain/drivetrain.h"

#include "commands/drivetrain/autoBalance.h"

DriveLevelCommand::DriveLevelCommand(Drivetrain* drivetrain) {
    c_drivetrain = drivetrain;

    c_autoBalance = new autoBalance(c_drivetrain);

    balancePID->SetTolerance(DEGREES_TO_RADIANS(2));
    balancePID->SetSetpoint(0);
    AddRequirements(c_drivetrain);
}


void DriveLevelCommand::Initialize() {
    c_drivetrain->setMotion(0, 0, 0); // make sure nothing is moving
    c_autoBalance->reset();
}
/*
void DriveLevelCommand::Execute() {
    double tiltMagnitude = std::sqrt( std::pow(c_drivetrain->getXTilt(), 2)
        + std::pow(c_drivetrain->getYTilt(), 2) ); // gets the angle from the floor
    double movementSpeed = 0;

    switch(currentAutoStage){
        case DRIVE_STATE::approachingChargingStation:
            if(tiltMagnitude > DEGREES_TO_RADIANS(5)){
                movementSpeed = 0;
                currentAutoStage = DRIVE_STATE::engagingChargingStation;
                break;
            }
            movementSpeed = 0.2;
            break;
        case DRIVE_STATE::engagingChargingStation:
            if(tiltMagnitude > DEGREES_TO_RADIANS(15)){
                movementSpeed = 0;
                currentAutoStage = DRIVE_STATE::balancingOnStation;
                break;
            }
            movementSpeed = 0.4;
        case DRIVE_STATE::balancingOnStation:
            if(tiltMagnitude < DEGREES_TO_RADIANS(2)){ // if less than 2 degrees, level
                movementSpeed = 0;
                currentAutoStage = DRIVE_STATE::done;
                break;
            }
            movementSpeed = balancePID->Calculate(tiltMagnitude);
            break;
        default:
            if(tiltMagnitude > DEGREES_TO_RADIANS(2)){
                currentAutoStage = DRIVE_STATE::balancingOnStation;
            }
            movementSpeed = 0;
            break;
    }

    c_drivetrain->setMotion(0, movementSpeed, 0);

}*/

void DriveLevelCommand::Execute() {
    c_drivetrain->setMotion(0, c_autoBalance->autoBalanceRoutine(), 0); // just move forwards
}

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

