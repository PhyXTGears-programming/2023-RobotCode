#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>


#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

class Calibration : public frc2::SubsystemBase {
public:
    Calibration();
    void Periodic();
    void turnOnMotors();
    void turnOffMotors();
    void resetEncoders();
private:
    double c_moveSpeed = 0.1;
    bool m_on = false;

    ctre::phoenix::motorcontrol::can::TalonFX * m_motors[4];
    ctre::phoenix::sensors::CANCoder * c_canCoder[4] = {
        new ctre::phoenix::sensors::CANCoder(21),
        new ctre::phoenix::sensors::CANCoder(22),
        new ctre::phoenix::sensors::CANCoder(23),
        new ctre::phoenix::sensors::CANCoder(24)
    };
};