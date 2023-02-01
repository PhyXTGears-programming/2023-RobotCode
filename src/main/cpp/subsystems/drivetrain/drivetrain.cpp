#include "subsystems/drivetrain/drivetrain.h"
#include "Mandatory.h"

#include "Constants.h"

#include <math.h>

/*
NOTE ON UNITS:

all math in here is based on the units of meters, radians and seconds.

there should not be any other units in here, so if there turns out to be,
it should be removed or put in a debug preprocessor.
*/

Drivetrain::Drivetrain(bool fieldOriented){
    this->m_fieldOriented = fieldOriented;
}

Drivetrain::Drivetrain(){
    this->m_fieldOriented = false;
}

Drivetrain::~Drivetrain(){
    // cleanup classes and prevent memory leaks
    for(int i=0; i<Constants::k_NumberOfSwerveModules;i+=1){
        delete c_wheels[i];
        m_wheelPositions[i][0] = 0;
        m_wheelPositions[i][1] = 0;
    }
}

void Drivetrain::Periodic(){
    //dont do anything. this is class is just a calculator and dispatcher
}

void Drivetrain::setupWheels(){
    //setup wheel classes with correct motors from toml file
}

void Drivetrain::calculateWheelPositionsAndSpeeds(){
    double motorDirectionAngle[Constants::k_NumberOfSwerveModules][2]; // index 0 is angle and index 1 is speed

    double maxSpeed;
    for (int i=0; i<Constants::k_NumberOfSwerveModules; i+=1){
        //combine the movement and turnign vectors
        double horizontal_motion = (Drivetrain::m_rotation * Drivetrain::m_wheelPositions[i][0]) + Drivetrain::m_strife;
        double verticle_motion = (Drivetrain::m_rotation * Drivetrain::m_wheelPositions[i][1]) + Drivetrain::m_forewards;

        // get the final angle of the module
        motorDirectionAngle[i][0] = atan2(horizontal_motion, verticle_motion);
        
        double speed = std::sqrt(std::pow(horizontal_motion, 2) + std::pow(verticle_motion, 2)); //flipped so that 0 is going towards the front
        
        motorDirectionAngle[i][1] = speed;

        if(speed > maxSpeed){
            maxSpeed = speed;
        }
    }

    //make sure the speed does not go above 1 (AKA: max power)
    if(maxSpeed > 1.0){
        for(int i = 0; i<Constants::k_NumberOfSwerveModules; i+=1){
            motorDirectionAngle[i][1] = motorDirectionAngle[i][1] / maxSpeed;
        }
    }
}

void Drivetrain::enableFieldCentric(){
    Drivetrain::m_fieldOriented = true;
}

void Drivetrain::disableFieldCentric(){
    Drivetrain::m_fieldOriented = false;
}

void Drivetrain::toggleFieldCentric(){
    Drivetrain::m_fieldOriented = !Drivetrain::m_fieldOriented;
}

bool Drivetrain::getFieldCentric(){
    return Drivetrain::m_fieldOriented;
}

void Drivetrain::setStrife(double x){
    Drivetrain::m_strife = x;
}

void Drivetrain::setForeward(double y){
    Drivetrain::m_forewards = y;
}

void Drivetrain::setRotation(double r){
    Drivetrain::m_rotation = r;
}

void Drivetrain::setMotion(double x, double y, double r){
    Drivetrain::m_strife = x;
    Drivetrain::m_forewards = y;
    Drivetrain::m_rotation = r;
}

double Drivetrain::getHeading(){
    return atan2(Drivetrain::m_strife, Drivetrain::m_forewards); //swapped x & y so forewards is 0 radians
}

double Drivetrain::getVelocity(){
    //using pythagorean to find the magnitude of the vector components (forewards and strife)
    return std::sqrt((std::pow(Drivetrain::m_strife, 2)+std::pow(Drivetrain::m_forewards, 2)));
}