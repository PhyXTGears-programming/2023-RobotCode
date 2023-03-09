#pragma once
#include "Mandatory.h"

class MotorInterfaces {
    public:
        /**
         * this is a function that will tell the motor to move in reference to power (aka between -1 and 1)
         * 
         * @param speed a percentage of the motor power
        */
        virtual void setMotion(double speed) {};

        /**
         * this will have the motor use PID to maintain a specific angle (requires absolute encoder)
         * 
         * @param radians the radian number to set the module to (using absolute encoder)
        */
        virtual void setRotation(double radians){};

        /**
         * this will output the velocity that the motor is going at in radians/second
        */
        virtual double getSensorVelocity(){return 0.0;};

        /**
         * @return rotation of motor in radians.
         */
        virtual double getSensorHeading(){return 0.0;};

        /**
         * Enable continous input on PID so motors (like steering) can turn full circle without reversing.
        */
        virtual void enableContinuousInput(){};

        /**
         * periodic function to update motors and pet the watchdog
        */
        virtual void Periodic(){};

        // have to have constructor and destructor for parent classes. set to nothing because they dont need anything
        // this is just so we can be assured that there are certain defined functions
        virtual ~MotorInterfaces() {};
        MotorInterfaces() {};
};
