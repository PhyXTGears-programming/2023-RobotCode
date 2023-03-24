#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/ctre_falcon.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/ControlMode.h>

#include <math.h>

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

    // GetSelectedSensorVelocity will output the number of encoder tick per second.
double CtreFalcon::getSensorVelocity() {
    // Falcon has 2048 Encoder ticks per revolution
    //
    // https://v5.docs.ctr-electronics.com/en/latest/ch14_MCSensor.html#sensor-resolution
    return (c_motor->GetSelectedSensorVelocity()/2048) * 2 * M_PI;
}

double CtreFalcon::getSensorHeading() {
    return 0.0;
}

void CtreFalcon::enableContinuousInput() {
    // Not implemented.
}

void CtreFalcon::setBrakeMode(bool shallBrake) {
    if (shallBrake) {
        c_motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    } else {
        c_motor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    }
}