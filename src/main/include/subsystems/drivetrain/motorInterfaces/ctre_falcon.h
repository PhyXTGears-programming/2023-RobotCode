#pragma once
#include "Mandatory.h"

#include "subsystems/drivetrain/motorInterfaces/motorInterfaces.h"

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>

class CtreFalcon : public MotorInterfaces{
    public:
        /**
         * creates and abstraction for the CTRE Falcon using CAN mode
         * 
         * @param canID the CAN ID to use on the controller
        */
        CtreFalcon(int canID);

        /**
         * this is a function that will make it in reference to power (aka between -1 and 1)
         * 
         * @param speed a percentage of the motor power
        */
        void setMotion(double speed) override;
    private:
        ctre::phoenix::motorcontrol::can::TalonFX * motor = nullptr;
};