#include "subsystems/drivetrain/swerveWheel.h"
#include "Mandatory.h"

#include "subsystems/drivetrain/swerveWheelTypes.h"

#include "subsystems/drivetrain/motorInterfaces/rev_sparkMaxBrushless.h"
#include "subsystems/drivetrain/motorInterfaces/ctre_falcon.h"

SwerveWheel::SwerveWheel(
    SwerveWheelTypes::SwerveWheelTypes turnMotor,
    SwerveWheelTypes::SwerveWheelTypes moveMotor,
    SwerveWheelTypes::SwerveWheelTypes encoder
) {
    switch (moveMotor.Vendor) {
        case VENDOR_REV_SPARKMAX:
            c_movementMotor = new RevSparkMaxBrushless(moveMotor.ID);
            break;
        case VENDOR_CTRE_FALCON:
            c_movementMotor = new CtreFalcon(moveMotor.ID);
            break;
    }


    switch (turnMotor.Vendor) {
        case VENDOR_REV_SPARKMAX:
            c_turningMotor = new RevSparkMaxBrushless(turnMotor.ID);
            break;
    }

    switch (encoder.Vendor) {
        case VENDOR_CTRE_CANCODER:
            break;
    }
}

void SwerveWheel::setVelocity(double s) {
    m_currentVelocity = s;
    c_movementMotor->setMotion(s);
}

void SwerveWheel::setMotion(double s, double r) {
    m_currentHeading = r;
    m_currentVelocity = s;
    c_movementMotor->setMotion(s);
}
