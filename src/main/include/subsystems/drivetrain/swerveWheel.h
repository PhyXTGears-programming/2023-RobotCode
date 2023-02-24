#pragma once
#include "Mandatory.h"

#include <frc2/command/SubsystemBase.h>

#include "subsystems/drivetrain/swerveWheelTypes.h"

#include "subsystems/drivetrain/motorInterfaces/motorInterfaces.h"

class SwerveWheel : public frc2::SubsystemBase {
    public:
        /**
         * code to create a swerve wheel instance
         * 
         * @param turnMotorID the CAN ID of the turn motor
         * @param moveMotorID the CAN ID of the drive motor
         * @param encoderID the CAN ID of the encoder
        */
        SwerveWheel(
            SwerveWheelTypes::SwerveWheelTypes turnMotor,
            SwerveWheelTypes::SwerveWheelTypes moveMotor,
            SwerveWheelTypes::SwerveWheelTypes encoder
        );

        void setHeading(double r);

        void setVelocity(double s);

        void setMotion(double s, double r);

        /**
         * used for the NavX global offset
        */
        void setHeadingOffset(double offset);

        double getHeading();

        double getVelocity();

        void Periodic() override;
    private:
        double m_currentVelocity = 0;
        double m_currentHeading = 0;
        double m_headingOffset = 0; //used with the NavX

        MotorInterfaces * c_movementMotor = nullptr;
        MotorInterfaces * c_turningMotor = nullptr;
};
