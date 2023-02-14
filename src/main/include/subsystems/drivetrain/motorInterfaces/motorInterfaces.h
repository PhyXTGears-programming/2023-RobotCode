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
         * periodic function to update motors and pet the watchdog
        */
        virtual void Periodic(){};

        // have to have constructor and destructor for parent classes. set to nothing because they dont need anything
        // this is just so we can be assured that there are certain defined functions
        virtual ~MotorInterfaces() {};
        MotorInterfaces() {};
};
