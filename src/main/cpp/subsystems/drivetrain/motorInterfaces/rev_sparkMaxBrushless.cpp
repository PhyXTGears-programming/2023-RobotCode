#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/rev_sparkMaxBrushless.h"

#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>


RevSparkMaxBrushless::RevSparkMaxBrushless(int canID){
    c_motor = new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless);
}

RevSparkMaxBrushless::RevSparkMaxBrushless(int motorCanID, int encoderCanID, frc2::PIDController pid){
    c_motor = new rev::CANSparkMax(motorCanID, rev::CANSparkMax::MotorType::kBrushless);

    RevSparkMaxBrushless::c_pidController = pid;
    RevSparkMaxBrushless::c_pidController.SetTolerance(0.1);
    // RevSparkMaxBrushless::c_pidController.EnableContinuousInput(-M_PI,M_PI);
    RevSparkMaxBrushless::c_pidControlled = true;

    RevSparkMaxBrushless::c_canCoder = new ctre::phoenix::sensors::CANCoder(encoderCanID);
    // RevSparkMaxBrushless::c_absoluteOffset = c_canCoder->GetAbsolutePosition();
}

}

void RevSparkMaxBrushless::setMotion(double speed){
    RevSparkMaxBrushless::c_motor->Set(speed);
}

void RevSparkMaxBrushless::setRotation(double radians){
}