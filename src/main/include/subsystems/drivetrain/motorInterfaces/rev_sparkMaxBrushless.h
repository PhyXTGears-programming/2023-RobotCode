#pragma once
#include "Mandatory.h"

#include "motorInterfaces.h"

#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix/sensors/CANCoder.h>

class RevSparkMaxBrushless : public MotorInterfaces {
    public:
        /**
         * creates and abstraction for the Rev Robotics Spark Max using CAN mode with a brushless motor
         * 
         * @param canID the CAN ID to use on the controller
        */
        RevSparkMaxBrushless(int canID);

        // RevSparkMaxBrushless();

        RevSparkMaxBrushless(int motorCanID, int encoderCanID, frc2::PIDController pid);

        /**
         * periodic function to update motors and pet the watchdog
        */
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
         * @return rotation of motor in radians.
        */
       double getSensorHeading() override;

        /**
         * Enable continous input on PID so motors (like steering) can turn full circle without reversing.
        */
        void enableContinuousInput() override;

        void setBrakeMode(bool shallBrake) override;
    private:
        double m_currentSpeedPercentage = 0;
        bool c_pidControlled = false;
        double c_absoluteOffset = 0;


        rev::CANSparkMax * c_motor = nullptr;
        frc2::PIDController c_pidController = frc2::PIDController{0,0,0};
        ctre::phoenix::sensors::CANCoder * c_canCoder = nullptr;
};
