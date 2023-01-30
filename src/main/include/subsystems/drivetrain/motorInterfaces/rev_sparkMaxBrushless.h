#pragma once
#include "Mandatory.h"

#include "motorInterfaces.h"

#include <rev/CANSparkMax.h>

class RevSparkMaxBrushless : public MotorInterfaces {
    public:
        /**
         * creates and abstraction for the Rev Robotics Spark Max using CAN mode with a brushless motor
         * 
         * @param canID the CAN ID to use on the controller
        */
        RevSparkMaxBrushless(int canID);

        /**
         * this is a function that will make it in reference to power (aka between -1 and 1)
         * 
         * @param speed a percentage of the motor power
        */
        void setMotion(double speed) override;
    private:
        rev::CANSparkMax * motor = nullptr;
};