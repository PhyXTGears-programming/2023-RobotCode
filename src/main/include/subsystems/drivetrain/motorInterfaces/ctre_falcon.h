#pragma once
#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/motorInterfaces.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

class CtreFalcon : public MotorInterfaces {
    public:
        /**
         * creates and abstraction for the CTRE Falcon using CAN mode
         * 
         * @param canID the CAN ID to use on the controller
        */
        CtreFalcon(int canID);

        CtreFalcon(int motorCanID, int encoderCanID);

        void Periodic() override;

        /**
         * this is a function that will tell the motor to move in reference to power (aka between -1 and 1)
         * 
         * @param speed a percentage of the motor power
        */
        void setMotion(double speed) override;

        /**
         * this will have the motor use PID to maintain a specific angle (requires absolute encoder)
         * 
         * @param radians the radian number to set the 
        */
        void setRotation(double radians) override;

        /**
         * Enable continous input on PID so motors (like steering) can turn full circle without reversing.
         */
        void enableContinuousInput() override;
    private:
        double m_currentSpeedPercentage = 0;

        ctre::phoenix::motorcontrol::can::TalonFX * c_motor = nullptr;
};
