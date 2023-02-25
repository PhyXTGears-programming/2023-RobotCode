#include "Mandatory.h"
#include "subsystems/drivetrain/odometry.h"
#include "subsystems/drivetrain/drivetrain.h"

Odometry::Odometry(Drivetrain* drivetrain){
    m_position.x = 0;
    m_position.y = 0;
    m_position.z = 0;
    m_drivetrain = drivetrain;
    m_previousTime = 0;
}

void Odometry::setPosition(double x, double y){
    m_position.x = x;
    m_position.y = y;
}

Point Odometry::getPosition(){
    return m_position;
}

void Odometry::Periodic(){
}