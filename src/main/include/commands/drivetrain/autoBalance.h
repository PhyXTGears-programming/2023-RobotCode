#pragma once

#include "Mandatory.h"

#include <frc/BuiltInAccelerometer.h>
#include <AHRS.h>
#include <cmath>
#include <math.h>

#include "subsystems/drivetrain/drivetrain.h"

class autoBalance{
    public:
        autoBalance(Drivetrain* drivetrain);
        void init();
        double getPitch();
        double getRoll();
        double getTilt();
        double autoBalanceRoutine();
        double scoreAndBalance();
        void reset();
        
    private:
        frc::BuiltInAccelerometer mAccel{};
        Drivetrain* c_drivetrain = nullptr;
        int state;
        int debounceCount;
        double yTiltOffset;
        double robotSpeedSlow;
        double robotSpeedFast;
        double onChargeStationDegree;
        double levelDegree;
        double debounceTime;
        double singleTapTime;
        double scoringBackUpTime;
        double doubleTapTime;
        double bangBangDegrees;
};