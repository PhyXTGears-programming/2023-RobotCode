#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/ctre_falcon.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

CtreFalcon::CtreFalcon(int canID) {
    motor = new ctre::phoenix::motorcontrol::can::TalonFX(canID);
}

void CtreFalcon::setMotion(double speed){
    motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}