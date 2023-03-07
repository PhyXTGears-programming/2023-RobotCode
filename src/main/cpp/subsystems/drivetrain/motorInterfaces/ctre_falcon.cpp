#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/ctre_falcon.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

CtreFalcon::CtreFalcon(int canID) {
    CtreFalcon::c_motor = new ctre::phoenix::motorcontrol::can::TalonFX(canID);
}

void CtreFalcon::Periodic() {
    CtreFalcon::c_motor->Set(
        ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
        CtreFalcon::m_currentSpeedPercentage
    );
}

void CtreFalcon::setMotion(double speed){
    CtreFalcon::m_currentSpeedPercentage = speed;
}

void CtreFalcon::setRotation(double radians) {
    // do nothing :)
    // at least for now...
}

double CtreFalcon::getRotation() {
    return 0.0;
}

void CtreFalcon::enableContinuousInput() {
    // Not implemented.
}