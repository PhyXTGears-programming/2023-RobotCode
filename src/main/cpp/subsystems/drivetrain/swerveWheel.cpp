#include "subsystems/drivetrain/swerveWheel.h"
#include "Mandatory.h"

#include "subsystems/drivetrain/swerveWheelTypes.h"

#include "subsystems/drivetrain/motorInterfaces/rev_sparkMaxBrushless.h"
#include "subsystems/drivetrain/motorInterfaces/ctre_falcon.h"

#include <frc/controller/PIDController.h>


SwerveWheel::SwerveWheel(
    SwerveWheelTypes::SwerveWheelTypes turnMotor,
    SwerveWheelTypes::SwerveWheelTypes moveMotor,
    SwerveWheelTypes::SwerveWheelTypes encoder
) {
    switch (moveMotor.Vendor){
        case VENDOR_REV_SPARKMAX:
            c_movementMotor = new RevSparkMaxBrushless(moveMotor.ID);
            break;
        case VENDOR_CTRE_FALCON:
            c_movementMotor = new CtreFalcon(moveMotor.ID);
            break;
    }


    switch (turnMotor.Vendor) {
        case VENDOR_REV_SPARKMAX:
            c_turningMotor = new RevSparkMaxBrushless(
                turnMotor.ID,
                encoder.ID,
                frc2::PIDController{0.2, 0, 0}
            );
            c_turningMotor->enableContinuousInput();
            break;
    }

    switch (encoder.Vendor) {
        case VENDOR_CTRE_CANCODER:
            break;
    }
}

void SwerveWheel::Periodic() {
    c_turningMotor->Periodic();
    c_movementMotor->Periodic();
}

void SwerveWheel::setVelocity(double s) {
    m_currentVelocity = s;
    c_movementMotor->setMotion(s);
}

void SwerveWheel::setHeading(double r) {
    m_currentHeading = r;
    c_turningMotor->setRotation(r);
}

void SwerveWheel::setMotion(double s, double r) {
    m_currentHeading = r;
    m_currentVelocity = s;
    c_movementMotor->setMotion(s);
    c_turningMotor->setRotation(r);
}

double SwerveWheel::getHeading(){
    return c_turningMotor->getSensorHeading();
}

double SwerveWheel::getVelocity(){
    /*
    6.12:1 drive motor to wheel gearing ratio
    4 in wheel diameter (0.1016 meters)
    converted to radius because:
    if w is the speed of the wheel in radians/second
    and r is the radius of the wheel
    anv v is the linear speed (how fast it is moving on the ground)
    v=wr
    */
    return (c_movementMotor->getSensorVelocity() / 6.12) * (0.1016/2);
}