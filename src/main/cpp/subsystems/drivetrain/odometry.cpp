#include "Mandatory.h"
#include "subsystems/drivetrain/odometry.h"
#include "subsystems/drivetrain/drivetrain.h"

#include <frc/Timer.h>
#include <units/time.h>

Odometry::Odometry(Drivetrain* drivetrain){
    m_position.x = 0;
    m_position.y = 0;
    m_position.z = 0;
    c_drivetrain = drivetrain;
    m_previousTime = 0;
}

void Odometry::setPosition(double x, double y){
    m_position.x = x;
    m_position.y = y;
}

Point Odometry::getPosition(){
    return m_position;
}

void Odometry::disableCalculation(){
    m_calculate = false;
}

void Odometry::enableCalculation(){
    m_calculate = true;
}

bool Odometry::calculationEnabled(){
    return m_calculate;
}

void Odometry::Periodic(){
    //init strife and forwards to 0
    double s = 0.0;
    double f = 0.0;

    double navxHeading = c_drivetrain->getFieldHeading();
    //go through each module and get the velocity vectors, then add them to the accumulator
    for(int i = 0; i<Constants::k_NumberOfSwerveModules;i++){
        s+=cos(c_drivetrain->getMovementHeading(i)+navxHeading)*c_drivetrain->getMovementVelocity(i);
        f+=sin(c_drivetrain->getMovementHeading(i)+navxHeading)*c_drivetrain->getMovementVelocity(i);
    }

    //divide the vectors by the number of modules to remove rotation vector 
    s = s/Constants::k_NumberOfSwerveModules;
    f = f/Constants::k_NumberOfSwerveModules;

    double currentTime = frc::Timer::GetFPGATimestamp().value();
    double deltaTime = (currentTime - m_previousTime);

    m_position.x = m_position.x + (s * deltaTime);
    m_position.y = m_position.y + (f * deltaTime);

    m_previousTime = currentTime; //make sure that it does not stay the same number forever
}