#pragma once
#include "Mandatory.h"

class MotorInterfaces {
    public:
        /**
         * this is a function that will make it in reference to power (aka between 0 and 1)
         * 
         * @param speed a percentage of the motor power
        */
        virtual void setMotion(double speed){};

        // have to have constructor and descructor for parent classes. set to nothing because they dont need anything
        // this is just so we can be assured that there are certain defined functions
        virtual ~MotorInterfaces(){};
        MotorInterfaces(){};
};