#include "Mandatory.h"
#include "subsystems/calibration/calibration.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>


Calibration::Calibration(){
    m_motors[0] = new ctre::phoenix::motorcontrol::can::TalonFX(1);
    m_motors[1] = new ctre::phoenix::motorcontrol::can::TalonFX(2);
    m_motors[2] = new ctre::phoenix::motorcontrol::can::TalonFX(3);
    m_motors[3] = new ctre::phoenix::motorcontrol::can::TalonFX(4);
}

void Calibration::Periodic(){
    if(m_on){
        for(int i=0; i<4;i++){
            m_motors[i]->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, c_moveSpeed);
        }
    } else {
        for(int i=0; i<4;i++){
            m_motors[i]->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        }
    }
}

void Calibration::turnOnMotors(){
    m_on = true;
}

void Calibration::resetEncoders(){
    for(int i=0;i<4;i++){
        c_canCoder[i]->ConfigMagnetOffset(0);
        double absPos =  c_canCoder[i]->GetAbsolutePosition();
        c_canCoder[i]->ConfigMagnetOffset(absPos);
    }
}