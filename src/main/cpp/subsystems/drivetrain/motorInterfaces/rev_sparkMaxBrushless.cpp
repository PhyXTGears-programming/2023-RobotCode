#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/rev_sparkMaxBrushless.h"

#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>


RevSparkMaxBrushless::RevSparkMaxBrushless(int canID) {
    c_motor = new rev::CANSparkMax(
        canID,
        rev::CANSparkMax::MotorType::kBrushless
    );
}

RevSparkMaxBrushless::RevSparkMaxBrushless(int motorCanID, int encoderCanID, frc2::PIDController pid) {
    c_motor = new rev::CANSparkMax(
        motorCanID,
        rev::CANSparkMax::MotorType::kBrushless
    );

    RevSparkMaxBrushless::c_pidController = pid;
    RevSparkMaxBrushless::c_pidController.SetTolerance(0.1);
    // RevSparkMaxBrushless::c_pidController.EnableContinuousInput(-M_PI,M_PI);
    RevSparkMaxBrushless::c_pidControlled = true;

    RevSparkMaxBrushless::c_canCoder = new ctre::phoenix::sensors::CANCoder(encoderCanID);
    // RevSparkMaxBrushless::c_absoluteOffset = c_canCoder->GetAbsolutePosition();
}

void RevSparkMaxBrushless::Periodic() {
    if (RevSparkMaxBrushless::c_pidControlled) {
        if (RevSparkMaxBrushless::c_pidController.AtSetpoint()) {
            RevSparkMaxBrushless::c_motor->Set(0);
        } else {
            double out = RevSparkMaxBrushless::c_pidController.Calculate(
                RevSparkMaxBrushless::c_canCoder->GetPosition()
                - RevSparkMaxBrushless::c_absoluteOffset
            );
            RevSparkMaxBrushless::c_motor->Set(-std::clamp(out, -0.5, 0.5)); //negative because the modules were going the wrong way. Should fix in the encoder firmware
        }
    } else {
        // RevSparkMaxBrushless::c_motor->Set(RevSparkMaxBrushless::m_currentSpeedPercentage);
    }
}

void RevSparkMaxBrushless::setMotion(double speed) {
    RevSparkMaxBrushless::m_currentSpeedPercentage = speed;
}

void RevSparkMaxBrushless::setRotation(double radians) {
    if (RevSparkMaxBrushless::c_pidControlled) {
        RevSparkMaxBrushless::c_pidController.SetSetpoint(radians);
    }
}
